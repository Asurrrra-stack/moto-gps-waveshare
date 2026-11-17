#include "motion_heading_sensor.h"
#include "motion_heading_math.hpp"

#include <cmath>

#include "bsp/esp-bsp.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// qmi8658.h defines M_PI globally. This adapter does not use that macro and
// removes it after the official component header to keep C++ <cmath> clean.
#include "qmi8658.h"
#ifdef M_PI
#undef M_PI
#endif

namespace {
constexpr char kTag[] = "motion_heading";
constexpr TickType_t kSampleDelay = pdMS_TO_TICKS(8);  // 125 Hz target.

std::uint64_t monotonic_ms() {
  return static_cast<std::uint64_t>(esp_timer_get_time()) / 1'000U;
}
}  // namespace

esp_err_t MotionHeadingSensor::start(SampleCallback callback, void* context) {
  if (callback == nullptr) return ESP_ERR_INVALID_ARG;
  if (started_) return ESP_ERR_INVALID_STATE;
  callback_ = callback;
  callback_context_ = context;
  // Sensor callbacks reach the presenter and LVGL adapter. Keep enough margin
  // for those nested frames even though the large map snapshot now remains in
  // PhoneNavBridge member storage.
  if (xTaskCreate(task_entry, "moto_qmi_heading", 8'192, this, 3, nullptr) !=
      pdPASS) {
    callback_ = nullptr;
    callback_context_ = nullptr;
    return ESP_ERR_NO_MEM;
  }
  started_ = true;
  return ESP_OK;
}

void MotionHeadingSensor::task_entry(void* context) {
  static_cast<MotionHeadingSensor*>(context)->run();
  vTaskDelete(nullptr);
}

void MotionHeadingSensor::run() {
  qmi8658_dev_t device{};
  const esp_err_t init = qmi8658_init(
      &device, bsp_i2c_get_handle(), QMI8658_ADDRESS_HIGH);
  if (init != ESP_OK) {
    ESP_LOGW(kTag, "QMI8658 unavailable; using phone course only: %s",
             esp_err_to_name(init));
    return;
  }
  ESP_ERROR_CHECK_WITHOUT_ABORT(
      qmi8658_set_accel_range(&device, QMI8658_ACCEL_RANGE_4G));
  ESP_ERROR_CHECK_WITHOUT_ABORT(
      qmi8658_set_accel_odr(&device, QMI8658_ACCEL_ODR_125HZ));
  ESP_ERROR_CHECK_WITHOUT_ABORT(
      qmi8658_set_gyro_range(&device, QMI8658_GYRO_RANGE_512DPS));
  ESP_ERROR_CHECK_WITHOUT_ABORT(
      qmi8658_set_gyro_odr(&device, QMI8658_GYRO_ODR_125HZ));
  qmi8658_set_accel_unit_mps2(&device, true);
  qmi8658_set_gyro_unit_dps(&device, true);

  // The old boot path blocked the display for up to four seconds waiting for
  // 120 "stable" samples and on this board often collected none at all (it
  // logged "bias 0.000 dps, 0 stable samples"), so nothing was ever calibrated.
  // The learner below starts immediately, converges within about a second of
  // standing still, and keeps tracking temperature drift afterwards.
  moto::heading::Gravity gravity;
  moto::heading::Bias bias;
  moto::heading::StillDetect stillness;
  ESP_LOGI(kTag, "QMI8658 yaw assist running at 125 Hz");

  qmi8658_data_t data{};
  int consecutive_failures = 0;
  std::uint64_t last_report_ms = 0;
  // Diagnostics follow the verdict, not the clock: a needle that starts or
  // stops creeping is a change of state and prints one line, while a board
  // sitting still says nothing until the five-minute keep-alive.
  int last_reported_still = -1;
  TickType_t next_sample_tick = xTaskGetTickCount();
  while (true) {
    bool ready = false;
    esp_err_t result = qmi8658_is_data_ready(&device, &ready);
    if (result == ESP_OK && ready) {
      result = qmi8658_read_sensor_data(&device, &data);
      if (result == ESP_OK) {
        consecutive_failures = 0;
        const float accel = moto::heading::magnitude3(
            data.accelX, data.accelY, data.accelZ);
        moto::heading::update_gravity(gravity, data.accelX, data.accelY,
                                     data.accelZ);
        // Judging stillness by spread, not by magnitude, is what lets the
        // learner keep working when an axis carries a large offset: a parked
        // board's rate is constant, a rider's is not. The magnitude cap below
        // only keeps a real turn from being learned as an offset.
        const bool steady = moto::heading::update_still(
            stillness, data.gyroX, data.gyroY, data.gyroZ, accel);
        const float raw_magnitude = moto::heading::magnitude3(
            data.gyroX, data.gyroY, data.gyroZ);
        const bool boot_window =
            bias.learned_frames < moto::heading::kBootLearnFrames;
        if (steady &&
            (boot_window ||
             raw_magnitude < moto::heading::kLearnRateCeilingDps)) {
          moto::heading::learn_bias(
              bias, data.gyroX, data.gyroY, data.gyroZ,
              boot_window ? moto::heading::kBootBiasGain
                          : moto::heading::kParkedBiasGain);
        }
        const float rate = moto::heading::heading_rate(gravity, bias,
                                                       data.gyroX, data.gyroY,
                                                       data.gyroZ);
        // Steady plus a near-zero projected rate is a parked board: whatever is
        // left on the gravity axis is offset, not rotation, so the heading must
        // not move. A real turn, however smooth, projects well above this.
        const bool still =
            steady && std::abs(rate) < moto::heading::kParkedRateCeilingDps;
        const std::uint64_t now_ms = monotonic_ms();
        // One line when the verdict settles on a new value, plus a five-minute
        // keep-alive so a silent log never hides a dead sensor. The five-second
        // floor matters: with the rate sitting near the parked ceiling the
        // verdict can flicker, and an unthrottled edge trigger would print a
        // burst of lines from a board that is actually sitting still.
        const bool flipped = last_reported_still != static_cast<int>(still);
        if ((flipped && now_ms - last_report_ms >= 5'000U) ||
            now_ms - last_report_ms >= 300'000U) {
          last_reported_still = static_cast<int>(still);
          last_report_ms = now_ms;
          ESP_LOGI(kTag,
                   "yaw still=%d grav=%.0f/%.0f/%.0f bias=%.2f/%.2f/%.2f "
                   "rate=%.2f raw=%.1f/%.1f/%.1f",
                   still ? 1 : 0, gravity.x, gravity.y, gravity.z, bias.x,
                   bias.y, bias.z, rate, data.gyroX, data.gyroY, data.gyroZ);
        }
        callback_(rate, now_ms, still, callback_context_);
      }
    }
    if (result != ESP_OK && ++consecutive_failures == 25) {
      ESP_LOGW(kTag, "QMI8658 read errors continue: %s",
               esp_err_to_name(result));
    }
    xTaskDelayUntil(&next_sample_tick, kSampleDelay);
  }
}

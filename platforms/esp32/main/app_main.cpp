#include <cstdint>

#include "ble_nav_transport_nimble.h"
#include "board_port.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_lv_adapter.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "moto_nav_presenter.hpp"
#include "moto_nav_ui.h"
#include "motion_heading_sensor.h"
#include "phone_nav_bridge.h"

namespace {
constexpr char kTag[] = "moto_gps";
constexpr std::uint64_t kPowerHoldMs = 3'000;
constexpr std::uint32_t kBatterySampleMs = 2'000;

const char* reset_reason_name(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:
      return "power-on";
    case ESP_RST_EXT:
      return "external pin";
    case ESP_RST_SW:
      return "software";
    case ESP_RST_PANIC:
      return "panic";
    case ESP_RST_INT_WDT:
      return "interrupt watchdog";
    case ESP_RST_TASK_WDT:
      return "task watchdog";
    case ESP_RST_WDT:
      return "other watchdog";
    case ESP_RST_DEEPSLEEP:
      return "deep sleep wake";
    case ESP_RST_BROWNOUT:
      return "brownout";
    case ESP_RST_SDIO:
      return "sdio";
    default:
      return "unknown";
  }
}

// Survives panic / watchdog resets, so a boot loop leaves a visible trail.
RTC_NOINIT_ATTR std::uint32_t g_boot_sequence;

moto::ble::AckStatus receive_phone_message(
    const moto::ble::ReassembledMessage& message, void* context) {
  return static_cast<PhoneNavBridge*>(context)->on_message(message);
}

void update_phone_link(bool active, void* context) {
  static_cast<PhoneNavBridge*>(context)->on_link_state(active);
}

void update_motion_heading(float heading_rate_dps, std::uint64_t sample_ms,
                           bool still, void* context) {
  static_cast<PhoneNavBridge*>(context)->on_imu_sample(heading_rate_dps,
                                                       sample_ms, still);
}

void demo_tick_task(void* context) {
  auto* bridge = static_cast<PhoneNavBridge*>(context);
  while (true) {
    const auto now_ms = static_cast<std::uint64_t>(esp_timer_get_time()) /
                        1'000U;
    bridge->update_demo(now_ms);
    // Match the display and route interpolation at 40 Hz. Keeping all three
    // clocks phase-compatible avoids periodic long frame gaps.
    vTaskDelay(pdMS_TO_TICKS(25));
  }
}

/**
 * AXP2101 pack sampling for the device-only POWER page. The navigation page
 * deliberately shows no battery, so this gauge is the only consumer.
 * Two seconds is slow enough to be invisible and fast enough that the reading
 * is current when the rider swipes to the page. Percent and voltage are blended
 * 3:1 over successive samples so one sagging reading under load cannot make the
 * gauge jump, while presence and charge direction are taken straight from the
 * PMU.
 */
void battery_task(void*) {
  moto_battery_state_t smoothed{};
  bool has_sample = false;
  while (true) {
    moto_battery_state_t sample{};
    const esp_err_t result = board_port_battery_read(&sample);
    if (result == ESP_OK) {
      // A pack cannot arrive or vanish mid-ride often, but when it does the
      // four-sample blend would take twenty seconds to reach the true gauge;
      // reset the history instead so the new reading shows immediately.
      const bool resync = !has_sample || smoothed.present != sample.present;
      if (resync) {
        smoothed = sample;
        has_sample = true;
      } else {
        smoothed.charging = sample.charging;
        smoothed.millivolts = static_cast<std::uint16_t>(
            (static_cast<std::uint32_t>(smoothed.millivolts) * 3U +
             sample.millivolts) / 4U);
        smoothed.percent = static_cast<std::uint8_t>(
            (static_cast<std::uint32_t>(smoothed.percent) * 3U +
             sample.percent) / 4U);
      }
      if (board_port_lock(UINT32_MAX)) {
        moto_nav_ui_set_battery(&smoothed);
        board_port_unlock();
      }
    } else if (result != ESP_ERR_INVALID_STATE) {
      ESP_LOGW(kTag, "battery sample failed: %s", esp_err_to_name(result));
    }
    vTaskDelay(pdMS_TO_TICKS(kBatterySampleMs));
  }
}

/**
 * One line every five minutes with the display worker's frame counter. A
 * counter that stops advancing while this line keeps printing separates "the
 * panel pipeline stalled" from "the whole board hung", which otherwise looks
 * identical from the outside: a frozen screen and a silent log. Five minutes
 * keeps that evidence available without making the ride log mostly this line.
 */
void heartbeat_task(void*) {
  constexpr std::uint32_t kIntervalMs = 300'000;
  unsigned long previous_frames = 0;
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(kIntervalMs));
    const unsigned long frames = board_port_display_frame_count();
    ESP_LOGI(kTag, "heartbeat: frames=%lu (+%lu per 5 min)", frames,
             frames - previous_frames);
    previous_frames = frames;
  }
}

void power_button_task(void*) {
  std::uint64_t pressed_since_ms = 0;
  // The AXP2101 needs roughly a one-second press to power the board on, so
  // the task usually starts while the user is still holding PWR. Arm the
  // hold-to-shutdown detector only after the line has been seen low once;
  // otherwise the tail of the power-on press is counted as a new 3-second
  // hold and the freshly booted unit switches itself off.
  bool released_once = false;
  while (true) {
    const std::uint64_t now_ms =
        static_cast<std::uint64_t>(esp_timer_get_time()) / 1'000U;
    if (!board_port_power_button_pressed()) {
      if (!released_once) {
        released_once = true;
        ESP_LOGI(kTag, "PWR ready: button released, hold detection armed");
      }
      pressed_since_ms = 0;
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }
    if (!released_once) {
      // Startup press still in progress; do not start the hold timer.
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }
    if (pressed_since_ms == 0) {
      pressed_since_ms = now_ms;
      ESP_LOGI(kTag, "PWR press started");
    } else if (now_ms - pressed_since_ms >= kPowerHoldMs) {
      ESP_LOGI(kTag, "PWR held for 3 seconds; requesting shutdown");
      if (board_port_lock(UINT32_MAX)) {
        moto_nav_ui_show_power_off_screen();
        board_port_unlock();
      }
      vTaskDelay(pdMS_TO_TICKS(300));
      const esp_err_t result = board_port_power_off();
      if (result != ESP_OK) {
        ESP_LOGE(kTag, "AXP2101 software power-off failed: %s",
                 esp_err_to_name(result));
      }

      // AXP2101 should remove the switched rails here. If this particular
      // board remains alive while USB is attached, blank the panel and enter
      // deep sleep after key release. GPIO3 high wakes it on the next press.
      vTaskDelay(pdMS_TO_TICKS(500));
      while (board_port_power_button_pressed()) {
        vTaskDelay(pdMS_TO_TICKS(20));
      }
      ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup_io(
          1ULL << 3, ESP_EXT1_WAKEUP_ANY_HIGH));
      esp_deep_sleep_start();
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
} // namespace

extern "C" void app_main(void) {
  static_assert(MOTO_DISPLAY_WIDTH == MOTO_UI_CANVAS_WIDTH &&
                    MOTO_DISPLAY_HEIGHT == MOTO_UI_CANVAS_HEIGHT,
                "The board and shared UI canvas must have one resolution");
  static_assert(LV_COLOR_DEPTH == MOTO_DISPLAY_BITS_PER_PIXEL,
                "Firmware and Web must both build the shared UI as RGB565");

  ESP_LOGI(kTag, "starting %dx%d RGB565 firmware target",
           MOTO_DISPLAY_WIDTH, MOTO_DISPLAY_HEIGHT);

  // NimBLE logs one "GATT procedure initiated" line for every notification it
  // sends, roughly one a second, which swamped every other line on the serial
  // port. Nothing this firmware relies on from the stack arrives below WARN.
  esp_log_level_set("NimBLE", ESP_LOG_WARN);

  // "It reboots by itself" has to be provable, not guessed: the reason module
  // below prints one line per boot, and brownout / watchdog / panic each read
  // differently. The counter survives a panic loop but resets on a power cycle.
  const esp_reset_reason_t reset_reason = esp_reset_reason();
  ++g_boot_sequence;
  ESP_LOGW(kTag, "reset reason: %s (code=%d) boot_sequence=%lu",
           reset_reason_name(reset_reason), static_cast<int>(reset_reason),
           static_cast<unsigned long>(g_boot_sequence));

  const esp_err_t init_result = board_port_init();
  if (init_result != ESP_OK) {
    ESP_LOGE(kTag, "board initialization stopped before shared UI startup: %s",
             esp_err_to_name(init_result));
    vTaskDelete(nullptr);
    return;
  }

  lv_display_t *const display = board_port_get_display();
  if (display == nullptr) {
    ESP_LOGE(kTag, "board port returned no LVGL display");
    vTaskDelete(nullptr);
    return;
  }

  if (lv_display_get_horizontal_resolution(display) != MOTO_DISPLAY_WIDTH ||
      lv_display_get_vertical_resolution(display) != MOTO_DISPLAY_HEIGHT ||
      lv_display_get_color_format(display) != LV_COLOR_FORMAT_RGB565) {
    ESP_LOGE(kTag, "board display violates the shared RGB565 portability "
                   "contract");
    vTaskDelete(nullptr);
    return;
  }

  if (!board_port_lock(UINT32_MAX)) {
    ESP_LOGE(kTag, "could not acquire LVGL lock");
    vTaskDelete(nullptr);
    return;
  }

  moto_nav_ui_show_boot_screen();
  // Commit the deliberately black first animation frame while the physical
  // panel is still hidden, then reveal it.  This removes the white frame that
  // used to leak from LVGL's default startup screen.
  lv_refr_now(display);
  const esp_err_t reveal_result = board_port_reveal_display();
  if (reveal_result != ESP_OK) {
    ESP_LOGE(kTag, "could not reveal boot animation: %s",
             esp_err_to_name(reveal_result));
  }
  board_port_unlock();

  // Give the display worker enough time to commit the monochrome power-on
  // frame before constructing the full production UI.
  vTaskDelay(pdMS_TO_TICKS(1'250));

  if (!board_port_lock(UINT32_MAX)) {
    ESP_LOGE(kTag, "could not reacquire LVGL lock after boot screen");
    vTaskDelete(nullptr);
    return;
  }

  moto_nav_ui_create();
  static moto::nav::NavPresenter presenter;
  static PhoneNavBridge phone_bridge(presenter);
  static BleNavTransport transport;
  // NavSnapshot contains the complete bounded roads/buildings window and is
  // several kilobytes.  A temporary here inflates app_main's stack frame for
  // the entire display/BSP startup call chain, which can trip the FreeRTOS
  // main-task canary before this line is even reached.  Keep the immutable
  // initial state in static storage just like the bridge's retained snapshots.
  static const moto::nav::NavSnapshot initial_snapshot{};
  presenter.update(initial_snapshot);
  presenter.apply_to_lvgl();
  moto_nav_ui_set_music_page_enabled(0);
  phone_bridge.install_ui_callbacks();
  // Commit the first production frame synchronously while the LVGL lock is
  // still ours, so the screen never sits on the boot scene waiting for a
  // timer-driven refresh.
  lv_refr_now(display);
  board_port_unlock();

  if (!phone_bridge.start_renderer()) {
    ESP_LOGE(kTag, "UI renderer startup failed; BLE was not started");
    vTaskDelete(nullptr);
    return;
  }

  phone_bridge.set_sender(BleNavTransport::send_from_bridge, &transport);
  transport.set_callbacks(receive_phone_message, update_phone_link,
                          &phone_bridge);

  // BLE_INIT occupies this core and the flash cache for ~100 ms. If it lands on
  // top of a 466x466 strip flush the panel's QSPI DMA underruns, the CO5300
  // reports ESP_ERR_INVALID_STATE once and then refuses every later draw, so
  // the screen freezes on the frame it happened to show while the rest of the
  // firmware looks healthy. Stop the display worker for the duration of the
  // controller start instead of racing it: pause() returns only after the
  // worker has acknowledged, which also means no transfer is in flight.
  const esp_err_t pause_result = esp_lv_adapter_pause(-1);
  if (pause_result != ESP_OK) {
    ESP_LOGW(kTag, "display worker pause failed: %s",
             esp_err_to_name(pause_result));
  }
  const esp_err_t ble_result = transport.start();
  esp_lv_adapter_resume();
  if (ble_result != ESP_OK) {
    ESP_LOGE(kTag, "BLE startup failed; display remains in offline mode: %s",
             esp_err_to_name(ble_result));
  }

  static MotionHeadingSensor motion_sensor;
  const esp_err_t motion_result = motion_sensor.start(update_motion_heading,
                                                      &phone_bridge);
  if (motion_result != ESP_OK) {
    ESP_LOGW(kTag, "QMI8658 heading assist could not start: %s",
             esp_err_to_name(motion_result));
  }

  // Demo generation fills the retained snapshot in place, but geometry and
  // LVGL projection still use deeper C++ call frames than a trivial task.
  // Depth is in bytes; 12 KiB with PSRAM backing leaves room for the log paths
  // that sit inside this task as well.
  if (xTaskCreateWithCaps(demo_tick_task, "moto_demo", 12'288, &phone_bridge, 2,
                          nullptr,
                          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
    ESP_LOGW(kTag, "navigation demo task could not start");
  }
  if (xTaskCreateWithCaps(power_button_task, "moto_power", 8'192, nullptr, 3,
                          nullptr,
                          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
    ESP_LOGW(kTag, "PWR long-hold task could not start");
  }
  // The sampler touches the LVGL tree (label text + arc value), so give it the
  // same headroom as the demo task rather than a minimal stack.
  if (xTaskCreateWithCaps(battery_task, "moto_batt", 12'288, nullptr, 2,
                          nullptr,
                          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
    ESP_LOGW(kTag, "battery sampler could not start");
  }
  // Heartbeat only reads a frame counter and logs it, so a small stack looked
  // generous. It is not: xTaskCreate takes the depth in BYTES on ESP-IDF, so
  // 2'048 meant a 2 KiB stack, while this build links the full newlib (nano
  // format off) and one ESP_LOGI expands to esp_log_writev -> vprintf ->
  // _vfprintf_r, which alone wants several KiB. After a few minutes of
  // heartbeats the printf frame walked past the guard, FreeRTOS reported "A
  // stack overflow in task moto_beat" and the board reset in a panic loop.
  // Every logging task is therefore given a real stack, allocated in PSRAM so
  // the already-tight internal DRAM (38 KiB free) is not the next failure.
  if (xTaskCreateWithCaps(heartbeat_task, "moto_beat", 16'384, nullptr, 1,
                          nullptr,
                          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
    ESP_LOGW(kTag, "heartbeat task could not start");
  }

  ESP_LOGI(kTag,
           "iPhone BLE + QMI heading -> NavPresenter -> shared LVGL running");
  vTaskDelete(nullptr);
}

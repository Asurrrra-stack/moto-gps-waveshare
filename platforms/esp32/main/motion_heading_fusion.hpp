#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

// Turns QMI8658 yaw-rate samples and phone course updates into one heading.
//
// This board has no magnetometer, so "north" is the phone's course whenever one
// is available and a local reference otherwise. Two rules keep the needle
// honest:
//
//  * No speed value may freeze the integration. The App reports speed 0.0 on
//    most navigating snapshots; the old code read that as "parked" and killed
//    the gyro on the first route snapshot, so the compass died the moment
//    navigation started and stayed dead until the next reboot.
//  * Only a credible course may correct. While stopped, Core Location still
//    emits a course that jumps tens of degrees, and a course more than a quarter
//    turn from the gyro heading is a stale fix rather than a bearing.
class MotionHeadingFusion {
 public:
  void reset() noexcept {
    initialized_ = false;
    has_phone_reference_ = false;
    has_moving_course_ = false;
    heading_deg_ = 0.0F;
    filtered_rate_dps_ = 0.0F;
    last_sample_ms_ = 0;
  }

  // speed_mps is the phone's reported speed; it stays the only moving signal
  // available here because the device cannot see GNSS progress directly.
  void anchor(float phone_heading_deg, float speed_mps,
              bool usable_fix) noexcept {
    if (!usable_fix || !std::isfinite(phone_heading_deg)) {
      return;
    }
    const float phone_speed = std::isfinite(speed_mps) ? speed_mps : 0.0F;
    const bool moving_course = phone_speed >= kCourseAnchorSpeedMps;
    const float phone = normalize(phone_heading_deg);
    // Gyro integration may start seconds before BLE supplies the first fix.
    // Its local zero must never be mistaken for an absolute phone reference.
    // Likewise, the first course after pulling away replaces a stationary
    // placeholder immediately, so an east/wouth route is not drawn behind us.
    if (!has_phone_reference_ || (moving_course && !has_moving_course_)) {
      heading_deg_ = phone;
      initialized_ = true;
      has_phone_reference_ = true;
      has_moving_course_ = moving_course;
      return;
    }
    if (!moving_course) {
      return;
    }
    const float error = shortest_delta(phone, heading_deg_);
    // A large error used to be discarded as a stale fix, which also threw away
    // every real turn of a quarter turn or more: the heading then never caught
    // up. Once the speed gate above has called this a moving course, the phone
    // is the only absolute reference, so any error is applied in full. Small
    // errors still creep in progressively to avoid a visible snap in a bend.
    const float correction = std::abs(error) > kMaximumCourseCorrectionDeg
                                 ? 1.0F
                                 : (std::abs(error) > 45.0F ? 0.5F : 0.25F);
    heading_deg_ = normalize(heading_deg_ + error * correction);
  }

  // yaw_rate_dps is positive for increasing compass heading (clockwise).
  // still marks a sample whose axes have not moved at all for 200 ms, which the
  // sensor derives from peak-to-peak spread. A parked board's residual offset is
  // integrated otherwise, and 1.5 dps of residual walks the needle a quarter
  // turn per minute while the screen merely stands on a table.
  // Returns true when a new display heading is available.
  bool integrate(float yaw_rate_dps, std::uint64_t sample_ms,
                 bool still = false) noexcept {
    if (!std::isfinite(yaw_rate_dps)) {
      return false;
    }
    if (!initialized_) {
      // Relative motion remains useful before the first trustworthy course;
      // zero degrees is explicitly just a temporary local reference.
      initialized_ = true;
      heading_deg_ = 0.0F;
    }
    if (last_sample_ms_ == 0) {
      last_sample_ms_ = sample_ms;
      return false;
    }
    if (sample_ms <= last_sample_ms_) {
      return false;
    }

    const std::uint64_t elapsed_ms = sample_ms - last_sample_ms_;
    last_sample_ms_ = sample_ms;
    // A still board contributes no rotation worth trusting. Freeze the heading
    // and drop the filter state so the first real move afterwards starts clean
    // instead of carrying a stale ramp.
    if (still) {
      filtered_rate_dps_ = 0.0F;
      return false;
    }
    if (elapsed_ms > kMaximumIntegrationGapMs) {
      filtered_rate_dps_ = 0.0F;
      return false;
    }

    const float bounded = std::clamp(yaw_rate_dps, -500.0F, 500.0F);
    // About 125 ms of smoothing at the 125 Hz sensor rate. Handlebar shake on
    // rough ground lands in the 5-10 Hz band at 10-20 dps peak, and a single
    // pole here only removes about three quarters of it; the deadband below
    // takes the rest. A real turn still settles inside a fifth of a second.
    filtered_rate_dps_ += (bounded - filtered_rate_dps_) * 0.06F;
    const float active_rate = std::abs(filtered_rate_dps_) < kDeadbandDps
                                  ? 0.0F
                                  : filtered_rate_dps_;
    if (active_rate == 0.0F) {
      return false;
    }
    heading_deg_ = normalize(
        heading_deg_ + active_rate * static_cast<float>(elapsed_ms) / 1'000.0F);
    return true;
  }

  [[nodiscard]] bool initialized() const noexcept { return initialized_; }
  [[nodiscard]] float heading_deg() const noexcept { return heading_deg_; }

 private:
  // 1.2 m/s is roughly walking pace: fast enough that Core Location is
  // producing a real course, slow enough that the App's 1.4-1.5 m/s reports
  // still qualify as moving.
  static constexpr float kCourseAnchorSpeedMps = 1.2F;
  // Errors at or below this value creep in progressively; beyond it the phone
  // course is applied in full, so any real turn converges even when the gyro
  // has drifted far away from the bearing.
  static constexpr float kMaximumCourseCorrectionDeg = 90.0F;
  // Rough-road handlebar shake reaches 10-20 dps peak, and after the 125 ms
  // smoothing above about a quarter of it survives. A real slow turn is tens
  // of dps (a 4 m radius at 10 km/h is about 40 dps), so this threshold only
  // removes the noise a rider never intended as input.
  static constexpr float kDeadbandDps = 2.8F;
  static constexpr std::uint64_t kMaximumIntegrationGapMs = 160;

  static float normalize(float value) noexcept {
    float result = std::fmod(value, 360.0F);
    if (result < 0.0F) result += 360.0F;
    return result;
  }

  static float shortest_delta(float target, float current) noexcept {
    float delta = normalize(target) - normalize(current);
    if (delta > 180.0F) delta -= 360.0F;
    if (delta < -180.0F) delta += 360.0F;
    return delta;
  }

  bool initialized_ = false;
  bool has_phone_reference_ = false;
  bool has_moving_course_ = false;
  float heading_deg_ = 0.0F;
  float filtered_rate_dps_ = 0.0F;
  std::uint64_t last_sample_ms_ = 0;
};

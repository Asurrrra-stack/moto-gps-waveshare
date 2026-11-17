#include "motion_heading_fusion.hpp"
#include "motion_heading_axis.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

int failures = 0;

#define CHECK(condition)                                                   \
  do {                                                                     \
    if (!(condition)) {                                                    \
      std::cerr << __FILE__ << ':' << __LINE__                             \
                << " CHECK failed: " #condition << '\n';                  \
      ++failures;                                                          \
    }                                                                      \
  } while (false)

void feed_rate(MotionHeadingFusion& fusion, float rate_dps,
               std::uint64_t start_ms, std::uint64_t end_ms) {
  for (std::uint64_t time = start_ms; time <= end_ms; time += 8) {
    fusion.integrate(rate_dps, time);
  }
}

void test_fast_gyro_turn_and_wrap() {
  MotionHeadingFusion fusion;
  fusion.anchor(350.0F, 8.0F, true);
  // 0.4 s at 90 dps is a 36 degree turn; the 125 ms smoothing holds back the
  // leading edge, so the expected result stays in the same band as before.
  feed_rate(fusion, 90.0F, 8, 408);
  CHECK(fusion.heading_deg() > 5.0F);
  CHECK(fusion.heading_deg() < 20.0F);
}

void test_stationary_phone_course_does_not_drag_handlebar_heading() {
  MotionHeadingFusion fusion;
  fusion.anchor(12.0F, 0.0F, true);
  fusion.anchor(210.0F, 0.2F, true);
  CHECK(std::abs(fusion.heading_deg() - 12.0F) < 0.01F);
}

void test_moving_phone_course_corrects_gyro_drift_progressively() {
  MotionHeadingFusion fusion;
  fusion.anchor(10.0F, 8.0F, true);
  CHECK(std::abs(fusion.heading_deg() - 10.0F) < 0.01F);
  // A credible moving course is corrected partially, not snapped onto.
  fusion.anchor(70.0F, 10.0F, true);
  CHECK(fusion.heading_deg() > 35.0F);
  CHECK(fusion.heading_deg() < 45.0F);
}

void test_first_phone_course_replaces_gyro_local_zero() {
  MotionHeadingFusion fusion;
  feed_rate(fusion, 90.0F, 8, 248);
  fusion.anchor(235.0F, 0.0F, true);
  CHECK(std::abs(fusion.heading_deg() - 235.0F) < 0.01F);
  // The first moving fix must also replace an old stationary heading in one
  // update instead of leaving the selected road behind the fixed rider arrow.
  fusion.anchor(90.0F, 8.0F, true);
  CHECK(std::abs(fusion.heading_deg() - 90.0F) < 0.01F);
  fusion.reset();
  feed_rate(fusion, -90.0F, 8, 248);
  fusion.anchor(180.0F, 12.0F, true);
  CHECK(std::abs(fusion.heading_deg() - 180.0F) < 0.01F);
}

void test_unusable_course_never_anchors() {
  MotionHeadingFusion fusion;
  fusion.anchor(90.0F, 12.0F, false);
  CHECK(!fusion.initialized());
  fusion.anchor(12.0F, 12.0F, true);
  fusion.anchor(180.0F, 12.0F, false);
  CHECK(std::abs(fusion.heading_deg() - 12.0F) < 0.01F);
}

void test_forward_facing_mount_yaw_does_not_reverse_at_zero_z() {
  const float front_tilt = clockwise_gravity_heading_rate(
      0.0F, 9.81F, 0.01F, 0.0F, -90.0F, 0.0F);
  const float back_tilt = clockwise_gravity_heading_rate(
      0.0F, 9.81F, -0.01F, 0.0F, -90.0F, 0.0F);
  CHECK(std::abs(front_tilt - 90.0F) < 0.01F);
  CHECK(std::abs(front_tilt - back_tilt) < 0.01F);
  // Rotating the mounting orientation leaves compass clockwise positive.
  CHECK(std::abs(clockwise_gravity_heading_rate(
      0.0F, 0.0F, 9.81F, 0.0F, 0.0F, -90.0F) - 90.0F) < 0.01F);
  CHECK(std::abs(clockwise_gravity_heading_rate(
      0.0F, 0.0F, -9.81F, 0.0F, 0.0F, 90.0F) - 90.0F) < 0.01F);
}

void test_large_sensor_gap_is_not_integrated() {
  MotionHeadingFusion fusion;
  fusion.anchor(42.0F, 5.0F, true);
  fusion.integrate(180.0F, 10);
  CHECK(!fusion.integrate(180.0F, 1'000));
  CHECK(std::abs(fusion.heading_deg() - 42.0F) < 0.01F);
}

float heading_with_optional_stale_sample(bool include_stale_sample) {
  MotionHeadingFusion fusion;
  fusion.anchor(90.0F, 8.0F, true);
  for (std::uint64_t time = 8; time <= 248; time += 8) {
    fusion.integrate(90.0F, time);
    if (include_stale_sample && time == 120) {
      fusion.integrate(90.0F, 116);
    }
  }
  return fusion.heading_deg();
}

void test_stale_timestamp_does_not_rewind_integration_clock() {
  const float clean = heading_with_optional_stale_sample(false);
  const float with_stale_sample = heading_with_optional_stale_sample(true);
  CHECK(std::abs(clean - with_stale_sample) < 0.01F);
}

void test_zero_phone_speed_never_freezes_gyro() {
  // Regression: the App reports speed 0.0 on most navigating snapshots. The
  // old anchor() read that as "parked" and froze gyro integration, so the
  // compass stopped the moment a route started and only a reboot revived it.
  MotionHeadingFusion fusion;
  fusion.anchor(30.0F, 0.0F, true);
  CHECK(std::abs(fusion.heading_deg() - 30.0F) < 0.01F);
  feed_rate(fusion, 90.0F, 8, 1'008);
  CHECK(fusion.heading_deg() > 30.0F);
}

void test_large_course_error_is_corrected_in_full() {
  // Regression: a course error beyond a quarter turn used to be dropped as a
  // stale fix, so a real 90+ degree turn never converged and the map stayed
  // behind the rider. With a credible moving course the phone heading wins at
  // any error size, while a zero-speed snapshot still never corrects.
  MotionHeadingFusion fusion;
  fusion.anchor(30.0F, 8.0F, true);
  fusion.anchor(210.0F, 10.0F, true);
  CHECK(std::abs(fusion.heading_deg() - 210.0F) < 0.5F);
  fusion.anchor(210.0F, 0.0F, true);
  CHECK(std::abs(fusion.heading_deg() - 210.0F) < 0.5F);
}

void test_handlebar_vibration_does_not_move_the_heading() {
  // Regression: with the old smoothing and a 0.9 dps deadband, the +/-1.2 dps
  // alternation a vibrating handlebar produces reached the display and the
  // needle jittered with every bump. Neither bikes, motorbikes nor cars should
  // see a heading change from it.
  MotionHeadingFusion fusion;
  fusion.anchor(45.0F, 8.0F, true);
  for (std::uint64_t time = 8; time <= 2'008; time += 8) {
    fusion.integrate((time % 16) == 0 ? 1.2F : -1.2F, time);
  }
  CHECK(std::abs(fusion.heading_deg() - 45.0F) < 1.0F);
}

void test_slow_real_turn_still_reaches_the_heading() {
  // The larger deadband and longer smoothing must not swallow a real turn:
  // 20 dps for one second has to move the heading by roughly 20 degrees.
  MotionHeadingFusion fusion;
  fusion.anchor(0.0F, 8.0F, true);
  feed_rate(fusion, 20.0F, 8, 1'008);
  CHECK(fusion.heading_deg() > 12.0F);
  CHECK(fusion.heading_deg() < 22.0F);
}

}  // namespace

int main() {
  test_fast_gyro_turn_and_wrap();
  test_stationary_phone_course_does_not_drag_handlebar_heading();
  test_moving_phone_course_corrects_gyro_drift_progressively();
  test_first_phone_course_replaces_gyro_local_zero();
  test_unusable_course_never_anchors();
  test_forward_facing_mount_yaw_does_not_reverse_at_zero_z();
  test_large_sensor_gap_is_not_integrated();
  test_stale_timestamp_does_not_rewind_integration_clock();
  test_zero_phone_speed_never_freezes_gyro();
  test_large_course_error_is_corrected_in_full();
  test_handlebar_vibration_does_not_move_the_heading();
  test_slow_real_turn_still_reaches_the_heading();

  if (failures != 0) {
    std::cerr << failures << " motion heading checks failed\n";
    return EXIT_FAILURE;
  }
  std::cout << "All motion heading checks passed\n";
  return EXIT_SUCCESS;
}

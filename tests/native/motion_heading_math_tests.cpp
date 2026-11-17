// Covers the sensor math that the board actually runs: gravity projection,
// per-axis bias, and the whole "stand the screen up, then lay it back down"
// pipeline through MotionHeadingFusion. The previous scalar-bias code passed
// every fusion test and still crept on a real bike, because this layer had no
// test at all.
#include "motion_heading_fusion.hpp"
#include "motion_heading_math.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

int failures = 0;

#define CHECK(condition)                                                   \
  do {                                                                     \
    if (!(condition)) {                                                    \
      std::cerr << __FILE__ << ':' << __LINE__                             \
                << " CHECK failed: " #condition << '\n';                   \
      ++failures;                                                          \
    }                                                                      \
  } while (false)

using moto::heading::Bias;
using moto::heading::Gravity;
using moto::heading::StillDetect;
using moto::heading::heading_rate;
using moto::heading::is_quiet;
using moto::heading::learn_bias;
using moto::heading::magnitude3;
using moto::heading::update_gravity;
using moto::heading::update_still;

// Mirrors MotionHeadingSensor::run() plus the bridge's integrate() call, so a
// test can replay a mounting change exactly as the board would see it.
struct Rig {
  Gravity gravity;
  Bias bias;
  StillDetect stillness;
  MotionHeadingFusion fusion;
  std::uint64_t now_ms = 0;

  void step(float ax, float ay, float az, float gx, float gy, float gz,
            std::uint64_t dt_ms = 8) {
    const float accel = magnitude3(ax, ay, az);
    update_gravity(gravity, ax, ay, az);
    const bool steady = update_still(stillness, gx, gy, gz, accel);
    const float raw_magnitude = magnitude3(gx, gy, gz);
    const bool boot_window =
        bias.learned_frames < moto::heading::kBootLearnFrames;
    if (steady &&
        (boot_window || raw_magnitude < moto::heading::kLearnRateCeilingDps)) {
      learn_bias(bias, gx, gy, gz,
                 boot_window ? moto::heading::kBootBiasGain
                             : moto::heading::kParkedBiasGain);
    }
    const float rate = heading_rate(gravity, bias, gx, gy, gz);
    const bool still = steady && std::abs(rate) < moto::heading::kParkedRateCeilingDps;
    fusion.integrate(rate, now_ms, still);
    now_ms += dt_ms;
  }

  float heading() const { return fusion.heading_deg(); }
};

void test_flat_board_reads_z_axis_rate() {
  Gravity gravity;
  Bias bias;
  update_gravity(gravity, 0.0F, 0.0F, 9.8F);
  CHECK(std::abs(heading_rate(gravity, bias, 0.0F, 0.0F, 10.0F) + 10.0F) < 0.01F);
}

void test_standing_board_reads_x_axis_rate() {
  // Screen on end: gravity lies along +x, so a turn around vertical is gyroX.
  Gravity gravity;
  Bias bias;
  update_gravity(gravity, 9.8F, 0.0F, 0.0F);
  CHECK(std::abs(heading_rate(gravity, bias, 10.0F, 0.0F, 0.0F) + 10.0F) < 0.01F);
}

void test_standing_upside_down_keeps_clockwise_sign() {
  // Gravity along -x: the same physical clockwise turn must stay positive.
  Gravity gravity;
  Bias bias;
  update_gravity(gravity, -9.8F, 0.0F, 0.0F);
  CHECK(std::abs(heading_rate(gravity, bias, -10.0F, 0.0F, 0.0F) + 10.0F) < 0.01F);
}

void test_per_axis_bias_cancels_standing_rate() {
  // The bug this fixes: a scalar offset learned on z leaves x creeping.
  Gravity gravity;
  Bias bias;
  update_gravity(gravity, 9.8F, 0.0F, 0.0F);
  bias.x = 0.6F;
  bias.z = 1.8F;
  CHECK(std::abs(heading_rate(gravity, bias, 0.6F, 0.0F, 0.0F)) < 0.01F);
}

void test_flat_bias_removes_all_three_axes() {
  Gravity gravity;
  Bias bias;
  update_gravity(gravity, 0.0F, 0.0F, 9.8F);
  for (int i = 0; i < 200; ++i) {
    learn_bias(bias, 0.6F, -0.4F, 1.8F);
  }
  CHECK(std::abs(bias.x - 0.6F) < 0.05F);
  CHECK(std::abs(bias.y + 0.4F) < 0.05F);
  CHECK(std::abs(bias.z - 1.8F) < 0.05F);
  CHECK(std::abs(heading_rate(gravity, bias, 0.6F, -0.4F, 1.8F)) < 0.05F);
}

void test_quiet_detection_sees_through_a_large_offset() {
  // Without this an axis can be too far off to be judged still, and therefore
  // never gets learned back: the learner locks itself out.
  Bias bias;
  bias.x = 6.0F;
  CHECK(is_quiet(bias, 6.0F, 0.0F, 0.0F, 9.8F));
}

void test_quiet_rejects_motion() {
  Bias bias;
  CHECK(!is_quiet(bias, 12.0F, 0.0F, 0.0F, 9.8F));
  CHECK(!is_quiet(bias, 0.0F, 0.0F, 0.0F, 4.0F));
}

void test_gravity_follows_a_remount_quickly() {
  Gravity gravity;
  update_gravity(gravity, 0.0F, 0.0F, 9.8F);
  for (int i = 0; i < 20; ++i) update_gravity(gravity, 9.8F, 0.0F, 0.0F);
  CHECK(gravity.x > 9.0F);
  CHECK(std::abs(gravity.z) < 1.0F);
}

void test_unusable_accel_keeps_the_stored_gravity() {
  Gravity gravity;
  update_gravity(gravity, 0.0F, 0.0F, 9.8F);
  const float before = gravity.z;
  CHECK(update_gravity(gravity, 0.0F, 0.0F, 2.0F) < 0.0F);
  CHECK(std::abs(gravity.z - before) < 0.01F);
}

void test_rate_is_never_dropped_before_the_first_accel_sample() {
  Gravity gravity;  // not initialized
  Bias bias;
  CHECK(std::abs(heading_rate(gravity, bias, 0.0F, 0.0F, 10.0F) + 10.0F) < 0.01F);
}

void test_standing_up_and_laying_back_down_keeps_the_heading() {
  // The reported fault, replayed end to end: flat, stand the screen up, hold
  // still for a minute, lay it back down. Nothing here is a real turn, so the
  // needle must not move.
  Rig rig;
  const float true_bias_x = 0.6F;
  const float true_bias_y = -0.4F;
  const float true_bias_z = 1.8F;

  // 3 s flat and still: learn the offsets.
  for (int i = 0; i < 375; ++i) {
    rig.step(0.0F, 0.0F, 9.8F, true_bias_x, true_bias_y, true_bias_z);
  }
  const float flat_heading = rig.heading();

  // 0.8 s of standing the screen up (pitch only, no turn about vertical).
  for (int i = 1; i <= 10; ++i) {
    const float t = static_cast<float>(i) / 10.0F;
    rig.step(9.8F * t, 0.0F, 9.8F * (1.0F - t), true_bias_x, true_bias_y,
             true_bias_z);
  }

  // 60 s standing still.
  for (int i = 0; i < 7'500; ++i) {
    rig.step(9.8F, 0.0F, 0.0F, true_bias_x, true_bias_y, true_bias_z);
  }
  const float standing_heading = rig.heading();

  // 0.8 s laying it back down, then 10 s flat.
  for (int i = 1; i <= 10; ++i) {
    const float t = static_cast<float>(i) / 10.0F;
    rig.step(9.8F * (1.0F - t), 0.0F, 9.8F * t, true_bias_x, true_bias_y,
             true_bias_z);
  }
  for (int i = 0; i < 1'250; ++i) {
    rig.step(0.0F, 0.0F, 9.8F, true_bias_x, true_bias_y, true_bias_z);
  }
  const float final_heading = rig.heading();

  CHECK(std::abs(standing_heading - flat_heading) < 3.0F);
  CHECK(std::abs(final_heading - flat_heading) < 3.0F);
}

void test_standing_turn_is_still_measured() {
  // Once upright, a real turn around vertical must still register at the right
  // size, not just stay quiet.
  Rig rig;
  for (int i = 0; i < 375; ++i) {
    rig.step(0.0F, 0.0F, 9.8F, 0.6F, -0.4F, 1.8F);
  }
  // Stand it up first and let the gravity estimate settle, exactly as it would
  // before the rider turns anything.
  for (int i = 1; i <= 10; ++i) {
    const float t = static_cast<float>(i) / 10.0F;
    rig.step(9.8F * t, 0.0F, 9.8F * (1.0F - t), 0.6F, -0.4F, 1.8F);
  }
  for (int i = 0; i < 250; ++i) {
    rig.step(9.8F, 0.0F, 0.0F, 0.6F, -0.4F, 1.8F);
  }
  const float start = rig.heading();
  for (int i = 0; i < 125; ++i) {  // 1 s at 20 dps around +x
    rig.step(9.8F, 0.0F, 0.0F, 0.6F + 20.0F, -0.4F, 1.8F);
  }
  const float delta = std::remainder(rig.heading() - start, 360.0F);
  // Sign follows the board axis (right-hand rule around +x), exactly like the
  // flat case does around +z. What must hold either way is the size. The
  // heading is normalized to 0..360, so a counter-clockwise turn crosses zero
  // and the raw subtraction reads as ~341 degrees; remainder() compares on
  // the shorter arc instead.
  CHECK(std::abs(delta) > 15.0F);
  CHECK(std::abs(delta) < 25.0F);
}

void test_parked_board_with_large_offset_does_not_creep() {
  // The reported fault, with the offset size the old gate could not handle.
  // "Quiet" used to mean every axis below 2.5 dps, so a 2.5 dps axis was never
  // learned at all: the offset survived and was integrated for as long as the
  // screen stood there. Spread-based stillness learns it and then refuses to
  // integrate, so two minutes parked must leave the heading alone.
  Rig rig;
  const float bx = 2.5F;
  const float by = -1.2F;
  const float bz = 1.9F;

  for (int i = 0; i < 375; ++i) {
    rig.step(0.0F, 0.0F, 9.8F, bx, by, bz);
  }
  for (int i = 1; i <= 10; ++i) {
    const float t = static_cast<float>(i) / 10.0F;
    rig.step(9.8F * t, 0.0F, 9.8F * (1.0F - t), bx, by, bz);
  }
  const float start = rig.heading();
  for (int i = 0; i < 15'000; ++i) {  // two minutes standing still
    rig.step(9.8F, 0.0F, 0.0F, bx, by, bz);
  }
  const float end = rig.heading();
  CHECK(std::abs(end - start) < 3.0F);
}

}  // namespace

int main() {
  test_flat_board_reads_z_axis_rate();
  test_standing_board_reads_x_axis_rate();
  test_standing_upside_down_keeps_clockwise_sign();
  test_per_axis_bias_cancels_standing_rate();
  test_flat_bias_removes_all_three_axes();
  test_quiet_detection_sees_through_a_large_offset();
  test_quiet_rejects_motion();
  test_gravity_follows_a_remount_quickly();
  test_unusable_accel_keeps_the_stored_gravity();
  test_rate_is_never_dropped_before_the_first_accel_sample();
  test_standing_up_and_laying_back_down_keeps_the_heading();
  test_standing_turn_is_still_measured();
  test_parked_board_with_large_offset_does_not_creep();

  if (failures != 0) {
    std::cerr << failures << " motion heading math checks failed\n";
    return EXIT_FAILURE;
  }
  std::cout << "motion heading math checks passed\n";
  return EXIT_SUCCESS;
}

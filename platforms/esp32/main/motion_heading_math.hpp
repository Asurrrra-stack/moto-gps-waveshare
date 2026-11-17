#pragma once

#include <algorithm>
#include <cmath>

// Pure heading math for the QMI8658 adapter, kept free of every ESP-IDF header
// so the host tests can exercise the exact code the board runs.
//
// The board has no magnetometer, so compass yaw comes from the gyroscope alone.
// That only stays honest if two things hold:
//
//  1. The rate is read around the *gravity* axis, not around the sensor's z
//     axis. Flat on the handlebars those are the same, but a screen standing on
//     end turns around x, and reading z there measures nothing.
//  2. Each gyro axis carries its own zero offset. One shared offset only
//     cancels the axis that happened to face gravity while it was measured,
//     which is why a standing screen crept slowly and never came back to where
//     it started.
namespace moto::heading {

struct Gravity {
  float x = 0.0F;
  float y = 0.0F;
  float z = 1.0F;
  bool initialized = false;
};

struct Bias {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  int learned_frames = 0;
};

constexpr float kMinimumGravity = 6.8F;
constexpr float kMaximumGravity = 12.8F;
// A sample counts as "still" when no axis of the corrected rate exceeds this
// and the accelerometer reads close to one g. The test runs on the corrected
// rate so a large offset can still learn itself away instead of locking the
// learner out.
constexpr float kQuietAngularDps = 2.5F;
constexpr float kQuietAccelMin = 8.2F;
constexpr float kQuietAccelMax = 11.4F;
// Fast learning for the first ~1.6 s after boot, slow afterwards, so a
// temperature shift is tracked without swallowing a real handlebar turn.
constexpr float kFastBiasGain = 0.02F;
constexpr int kFastBiasFrames = 200;
constexpr float kSlowBiasGain = 0.002F;
// Same gravity vector, or has the board been re-mounted? 0.995 is about 6
// degrees: looser than this and the estimate stalls a third of the way through
// the move, so every reading is projected onto a direction that is still wrong.
constexpr float kFastGravityCos = 0.995F;

inline float magnitude3(float x, float y, float z) {
  return std::sqrt(x * x + y * y + z * z);
}

// Follows the gravity direction. Returns the cosine between the new sample and
// the stored estimate, or -1 when the sample cannot be trusted (which also
// leaves the stored estimate untouched).
inline float update_gravity(Gravity& gravity, float accel_x, float accel_y,
                            float accel_z) {
  const float length = magnitude3(accel_x, accel_y, accel_z);
  if (!std::isfinite(length) || length < kMinimumGravity ||
      length > kMaximumGravity) {
    return -1.0F;
  }
  if (!gravity.initialized) {
    gravity.x = accel_x;
    gravity.y = accel_y;
    gravity.z = accel_z;
    gravity.initialized = true;
    return 1.0F;
  }
  const float stored = magnitude3(gravity.x, gravity.y, gravity.z);
  const float dot = accel_x * gravity.x + accel_y * gravity.y +
                    accel_z * gravity.z;
  const float cos_angle = stored > 0.01F ? dot / (length * stored) : 1.0F;
  // Re-mount poses arrive as a large disagreement; following them quickly is
  // what stops the turn rate from being projected onto the wrong axis while
  // the board is being stood up.
  const float blend = cos_angle < kFastGravityCos ? 0.25F : 0.02F;
  gravity.x += (accel_x - gravity.x) * blend;
  gravity.y += (accel_y - gravity.y) * blend;
  gravity.z += (accel_z - gravity.z) * blend;
  return cos_angle;
}

// Compass yaw rate in degrees per second, positive clockwise seen from above:
// the bias-corrected angular velocity projected onto the gravity axis. Before
// the first accelerometer sample it falls back to the flat-board axis so a
// gyro reading is never silently dropped.
inline float heading_rate(const Gravity& gravity, const Bias& bias,
                          float gyro_x, float gyro_y, float gyro_z) {
  const float cx = gyro_x - bias.x;
  const float cy = gyro_y - bias.y;
  const float cz = gyro_z - bias.z;
  if (!gravity.initialized) return -cz;
  const float stored = magnitude3(gravity.x, gravity.y, gravity.z);
  if (stored < 0.01F) return -cz;
  const float gx = gravity.x / stored;
  const float gy = gravity.y / stored;
  const float gz = gravity.z / stored;
  return -(cx * gx + cy * gy + cz * gz);
}

// "Still" is judged after the offsets are removed, so an axis whose offset has
// grown past the threshold can still be learned and learned back away.
inline bool is_quiet(const Bias& bias, float gyro_x, float gyro_y,
                     float gyro_z, float accel_mps2) {
  const float cx = gyro_x - bias.x;
  const float cy = gyro_y - bias.y;
  const float cz = gyro_z - bias.z;
  return std::abs(cx) < kQuietAngularDps && std::abs(cy) < kQuietAngularDps &&
         std::abs(cz) < kQuietAngularDps && accel_mps2 > kQuietAccelMin &&
         accel_mps2 < kQuietAccelMax;
}

inline void learn_bias(Bias& bias, float gyro_x, float gyro_y, float gyro_z,
                       float gain) {
  bias.x += (gyro_x - bias.x) * gain;
  bias.y += (gyro_y - bias.y) * gain;
  bias.z += (gyro_z - bias.z) * gain;
  // Counted unconditionally: the boot window is measured in frames since power
  // on, not in frames it happened to learn.
  ++bias.learned_frames;
}

inline void learn_bias(Bias& bias, float gyro_x, float gyro_y, float gyro_z) {
  const float gain = bias.learned_frames < kFastBiasFrames ? kFastBiasGain
                                                           : kSlowBiasGain;
  learn_bias(bias, gyro_x, gyro_y, gyro_z, gain);
}

// ---------------------------------------------------------------------------
// Steadiness, judged by spread and never by magnitude.
//
// Measured on this board at rest: about 5.6 / -4.0 / -0.3 dps, a stable
// three-axis offset whose magnitude (6.9 dps) exceeds any real slow turn. A rule
// that calls a sample "still" because the rate is small therefore discards the
// offset, and the offset is precisely what creeps: flat on the bars it projects
// to 0.3 dps and vanishes inside the deadband, but stood on end it projects to
// 5.6 dps and walks the needle right round.
//
// What separates parked from moving is *spread* over a short window. A rider
// changes the rate continuously; a parked board only shows sensor noise around
// a fixed value. So `still.still` here means "steady", and the caller combines
// it with the projected rate before freezing or learning.
struct StillDetect {
  float lo_x = 0.0F, hi_x = 0.0F;
  float lo_y = 0.0F, hi_y = 0.0F;
  float lo_z = 0.0F, hi_z = 0.0F;
  float lo_a = 0.0F, hi_a = 0.0F;
  int frames = 0;
  bool primed = false;
  bool still = false;
};

constexpr int kStillWindowFrames = 25;      // 200 ms at 125 Hz.
constexpr float kStillSpreadDps = 1.3F;     // Peak-to-peak per axis.
constexpr float kStillAccelSpread = 0.45F;  // m/s^2 spread over the window.
// Learning only runs while the reading is close to a plausible sensor offset;
// 8 dps sits above the 6.9 dps measured here and well below any real turn.
constexpr float kLearnRateCeilingDps = 8.0F;
// The first seconds after boot are the best chance to capture a large offset:
// the board is usually sitting still and nothing it reports can be a turn.
constexpr int kBootLearnFrames = 300;
constexpr float kBootBiasGain = 0.05F;
constexpr float kParkedBiasGain = 0.02F;
// The projected rate at or below which the heading is allowed to stand still.
// Bounded learning converges well inside it, so a parked board ends up frozen.
constexpr float kParkedRateCeilingDps = 1.5F;

inline void seed_still_window(StillDetect& still, float gyro_x, float gyro_y,
                              float gyro_z, float accel_mps2) {
  still.lo_x = still.hi_x = gyro_x;
  still.lo_y = still.hi_y = gyro_y;
  still.lo_z = still.hi_z = gyro_z;
  still.lo_a = still.hi_a = accel_mps2;
  still.frames = 1;
  still.primed = true;
}

// Feeds one sample and returns the current steadiness verdict, which only
// changes when a window closes.
inline bool update_still(StillDetect& still, float gyro_x, float gyro_y,
                         float gyro_z, float accel_mps2) {
  if (!still.primed) {
    seed_still_window(still, gyro_x, gyro_y, gyro_z, accel_mps2);
    still.still = false;
    return false;
  }
  still.lo_x = std::min(still.lo_x, gyro_x);
  still.hi_x = std::max(still.hi_x, gyro_x);
  still.lo_y = std::min(still.lo_y, gyro_y);
  still.hi_y = std::max(still.hi_y, gyro_y);
  still.lo_z = std::min(still.lo_z, gyro_z);
  still.hi_z = std::max(still.hi_z, gyro_z);
  still.lo_a = std::min(still.lo_a, accel_mps2);
  still.hi_a = std::max(still.hi_a, accel_mps2);
  ++still.frames;
  if (still.frames < kStillWindowFrames) return still.still;
  still.still = (still.hi_x - still.lo_x) < kStillSpreadDps &&
                (still.hi_y - still.lo_y) < kStillSpreadDps &&
                (still.hi_z - still.lo_z) < kStillSpreadDps &&
                (still.hi_a - still.lo_a) < kStillAccelSpread &&
                accel_mps2 > kQuietAccelMin && accel_mps2 < kQuietAccelMax;
  seed_still_window(still, gyro_x, gyro_y, gyro_z, accel_mps2);
  return still.still;
}

}  // namespace moto::heading

#include "moto_nav_presenter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <string>

namespace moto::nav {
namespace {

constexpr double kEarthRadiusM = 6'371'000.0;
constexpr double kPi = 3.14159265358979323846;
constexpr double kLegacyCanvasSize = 360.0;
constexpr double kUiScale =
    static_cast<double>(MOTO_UI_CANVAS_WIDTH) / kLegacyCanvasSize;
constexpr double kVehicleX = MOTO_UI_CANVAS_WIDTH / 2.0;
constexpr double kVehicleY = 196.0 * kUiScale;
// A navigation minimap is a spatial frame of reference, so it must not jump
// between zoom levels as a maneuver counter crosses an arbitrary threshold.
// This fixed street-scale view shows roughly 400 m ahead on the 466 px round
// display and is shared by the selected route and all context roads.
constexpr double kMapPixelsPerMeter = 0.44 * kUiScale;

double radians(double degrees) { return degrees * kPi / 180.0; }

moto_ui_mode_t map_mode(const NavSnapshot& snapshot) {
  switch (snapshot.state) {
    case NavState::Idle:
    case NavState::Acquiring:
    case NavState::Planning:
      return MOTO_UI_ACQUIRING_FIX;
    case NavState::Navigating:
      return snapshot.network == NetworkState::Offline
                 ? MOTO_UI_OFFLINE
                 : MOTO_UI_NAVIGATING;
    case NavState::Rerouting:
      return MOTO_UI_REROUTING;
    case NavState::Arrived:
      return MOTO_UI_ARRIVED;
  }
  return MOTO_UI_ACQUIRING_FIX;
}

moto_ui_page_t map_page(DisplayPage page) {
  switch(page) {
    case DisplayPage::Navigation: return MOTO_UI_PAGE_NAVIGATION;
    case DisplayPage::Speed: return MOTO_UI_PAGE_SPEED;
    case DisplayPage::Compass: return MOTO_UI_PAGE_COMPASS;
    case DisplayPage::Music: return MOTO_UI_PAGE_MUSIC;
  }
  return MOTO_UI_PAGE_NAVIGATION;
}

moto_maneuver_t map_maneuver(ManeuverType maneuver) {
  switch (maneuver) {
    case ManeuverType::SlightLeft:
      return MOTO_MANEUVER_SLIGHT_LEFT;
    case ManeuverType::Left:
    case ManeuverType::SharpLeft:
      return MOTO_MANEUVER_LEFT;
    case ManeuverType::SlightRight:
      return MOTO_MANEUVER_SLIGHT_RIGHT;
    case ManeuverType::Right:
    case ManeuverType::SharpRight:
    case ManeuverType::Exit:
      return MOTO_MANEUVER_RIGHT;
    case ManeuverType::UTurnLeft:
    case ManeuverType::UTurnRight:
      return MOTO_MANEUVER_UTURN;
    case ManeuverType::Roundabout:
      return MOTO_MANEUVER_ROUNDABOUT;
    case ManeuverType::Arrive:
      return MOTO_MANEUVER_ARRIVE;
    case ManeuverType::Unknown:
    case ManeuverType::Continue:
      return MOTO_MANEUVER_STRAIGHT;
  }
  return MOTO_MANEUVER_STRAIGHT;
}

moto_traffic_t map_traffic(TrafficLevel traffic) {
  switch (traffic) {
    case TrafficLevel::FreeFlow:
      return MOTO_TRAFFIC_CLEAR;
    case TrafficLevel::Slow:
      return MOTO_TRAFFIC_SLOW;
    case TrafficLevel::Congested:
      return MOTO_TRAFFIC_CONGESTED;
    case TrafficLevel::Severe:
      return MOTO_TRAFFIC_SEVERE;
    case TrafficLevel::Unknown:
      return MOTO_TRAFFIC_UNKNOWN;
  }
  return MOTO_TRAFFIC_UNKNOWN;
}

// The phone owns routing and only ever labels a movement as "left" or "right"
// (plus its own arrive / continue states), so the sharper glyphs -- straight,
// slight, u-turn, roundabout -- would never reach the panel. Everything below
// re-derives the movement on the device from data the snapshot already carries:
// the instruction text and the route-view polyline. No protocol field, byte
// order or enum value is touched, so the phone stays unchanged.
bool instruction_has(const std::string& text, const char* needle) {
  return text.find(needle) != std::string::npos;
}

bool instruction_has_any(const std::string& text,
                         std::initializer_list<const char*> needles) {
  for (const char* needle : needles) {
    if (instruction_has(text, needle)) {
      return true;
    }
  }
  return false;
}

// Chinese keywords are written as UTF-8 byte escapes so the match does not
// depend on how the compiler is told to interpret the source file. The comment
// next to each literal is the readable form.
ManeuverType maneuver_from_instruction(const std::string& instruction,
                                       std::uint8_t roundabout_exit) {
  if (roundabout_exit > 0 ||
      instruction_has_any(instruction,
                          {"\xE7\x8E\xAF\xE5\xB2\x9B",   // 环岛
                           "\xE7\xBB\x95\xE5\xB2\x9B",   // 绕岛
                           "roundabout", "rotary"})) {
    return ManeuverType::Roundabout;
  }
  if (instruction_has_any(instruction,
                          {"\xE6\x8E\x89\xE5\xA4\xB4",   // 掉头
                           "\xE8\xB0\x83\xE5\xA4\xB4",   // 调头
                           "u-turn", "uturn"})) {
    return ManeuverType::UTurnLeft;
  }
  if (instruction_has_any(instruction,
                          {"\xE5\x88\xB0\xE8\xBE\xBE",   // 到达
                           "\xE6\x8A\xB5\xE8\xBE\xBE",   // 抵达
                           "arrive", "destination"})) {
    return ManeuverType::Arrive;
  }
  if (instruction_has_any(instruction,
                          {"\xE7\xA8\x8D\xE5\x90\x91\xE5\x8F\xB3",  // 稍向右
                           "\xE5\x90\x91\xE5\x8F\xB3\xE5\x89\x8D"   // 向右前
                                "\xE6\x96\xB9",
                           "\xE9\x9D\xA0\xE5\x8F\xB3",  // 靠右
                           "slight right", "keep right"})) {
    return ManeuverType::SlightRight;
  }
  if (instruction_has_any(instruction,
                          {"\xE7\xA8\x8D\xE5\x90\x91\xE5\xB7\xA6",  // 稍向左
                           "\xE5\x90\x91\xE5\xB7\xA6\xE5\x89\x8D"   // 向左前
                                "\xE6\x96\xB9",
                           "\xE9\x9D\xA0\xE5\xB7\xA6",  // 靠左
                           "slight left", "keep left"})) {
    return ManeuverType::SlightLeft;
  }
  if (instruction_has_any(instruction,
                          {"\xE7\x9B\xB4\xE8\xA1\x8C",          // 直行
                           "\xE7\xBB\xA7\xE7\xBB\xAD\xE7\x9B\xB4",  // 继续直
                           "\xE4\xBF\x9D\xE6\x8C\x81",          // 保持
                           "straight", "continue"})) {
    return ManeuverType::Continue;
  }
  if (instruction_has_any(instruction,
                          {"\xE5\xB7\xA6\xE8\xBD\xAC",          // 左转
                           "\xE5\x90\x91\xE5\xB7\xA6",          // 向左
                           "turn left", "left"})) {
    return ManeuverType::Left;
  }
  if (instruction_has_any(instruction,
                          {"\xE5\x8F\xB3\xE8\xBD\xAC",          // 右转
                           "\xE5\x90\x91\xE5\x8F\xB3",          // 向右
                           "turn right", "right"})) {
    return ManeuverType::Right;
  }
  return ManeuverType::Unknown;
}

double point_distance_m(const Gcj02Point& a, const Gcj02Point& b) {
  const double mean_latitude = radians((a.latitude_deg + b.latitude_deg) / 2.0);
  const double dx =
      radians(b.longitude_deg - a.longitude_deg) * std::cos(mean_latitude);
  const double dy = radians(b.latitude_deg - a.latitude_deg);
  return kEarthRadiusM * std::sqrt(dx * dx + dy * dy);
}

// Compass bearing, 0 = north, 90 = east, so a clockwise (positive) change is
// a right-hand movement.
bool bearing_deg(const Gcj02Point& from, const Gcj02Point& to, double* out) {
  const double mean_latitude =
      radians((from.latitude_deg + to.latitude_deg) / 2.0);
  const double dx =
      radians(to.longitude_deg - from.longitude_deg) * std::cos(mean_latitude);
  const double dy = radians(to.latitude_deg - from.latitude_deg);
  if (dx == 0.0 && dy == 0.0) {
    return false;
  }
  *out = std::atan2(dx, dy) * 180.0 / kPi;
  return true;
}

// Bends the route 300 m ahead into one of the four glyph families. The window
// is short, so this only ever runs when the corner is actually inside it; an
// out-of-window target returns Unknown and the phone's own label survives.
ManeuverType maneuver_from_geometry(const NavSnapshot& snapshot,
                                    double distance_to_maneuver_m,
                                    double baseline_m = 0.0) {
  const std::uint8_t count = snapshot.route_view_point_count;
  if (!snapshot.has_route_view || count < 3) {
    return ManeuverType::Unknown;
  }
  std::size_t vertex = 0;
  double cumulative_m = 0.0;
  double best_gap_m = std::numeric_limits<double>::infinity();
  for (std::size_t index = 1; index < count; ++index) {
    cumulative_m += point_distance_m(snapshot.route_view_points[index - 1],
                                     snapshot.route_view_points[index]);
    const double gap = std::fabs(cumulative_m - distance_to_maneuver_m);
    if (gap < best_gap_m) {
      best_gap_m = gap;
      vertex = index;
    }
  }
  if (vertex == 0 || vertex + 1 >= count || best_gap_m > 120.0) {
    return ManeuverType::Unknown;
  }
  // One segment by default. A caller may ask for a wider baseline: a rounded or
  // chamfered right angle is drawn as two 30-45 degree segments, and a
  // one-segment reading undercuts it into a slight bend.
  std::size_t incoming_index = vertex - 1;
  std::size_t outgoing_index = vertex + 1;
  if (baseline_m > 0.0) {
    double leg_m = 0.0;
    for (std::size_t index = vertex - 1; index > 0; --index) {
      leg_m += point_distance_m(snapshot.route_view_points[index - 1],
                                snapshot.route_view_points[index]);
      incoming_index = index - 1;
      if (leg_m >= baseline_m) break;
    }
    leg_m = 0.0;
    for (std::size_t index = vertex + 1; index + 1 < count; ++index) {
      leg_m += point_distance_m(snapshot.route_view_points[index],
                                snapshot.route_view_points[index + 1]);
      outgoing_index = index + 1;
      if (leg_m >= baseline_m) break;
    }
  }
  double incoming = 0.0;
  double outgoing = 0.0;
  if (!bearing_deg(snapshot.route_view_points[incoming_index],
                   snapshot.route_view_points[vertex], &incoming) ||
      !bearing_deg(snapshot.route_view_points[vertex],
                   snapshot.route_view_points[outgoing_index], &outgoing)) {
    return ManeuverType::Unknown;
  }
  double delta = outgoing - incoming;
  while (delta > 180.0) delta -= 360.0;
  while (delta < -180.0) delta += 360.0;
  const double magnitude = std::fabs(delta);
  const bool right_hand = delta > 0.0;
  if (magnitude < 15.0) {
    return ManeuverType::Continue;
  }
  if (magnitude < 45.0) {
    return right_hand ? ManeuverType::SlightRight : ManeuverType::SlightLeft;
  }
  if (magnitude < 145.0) {
    return right_hand ? ManeuverType::Right : ManeuverType::Left;
  }
  return right_hand ? ManeuverType::UTurnRight : ManeuverType::UTurnLeft;
}

bool is_left_hand(ManeuverType type) {
  return type == ManeuverType::Left || type == ManeuverType::SlightLeft ||
         type == ManeuverType::SharpLeft || type == ManeuverType::UTurnLeft;
}

bool is_right_hand(ManeuverType type) {
  return type == ManeuverType::Right || type == ManeuverType::SlightRight ||
         type == ManeuverType::SharpRight || type == ManeuverType::UTurnRight;
}

// How hard a label reads the corner: 0 straight/none, 1 slight, 2 turn,
// 3 sharp, 4 reversal. Only used to let the geometry sharpen a weak label.
int maneuver_rank(ManeuverType type) {
  switch (type) {
    case ManeuverType::SlightLeft:
    case ManeuverType::SlightRight:
      return 1;
    case ManeuverType::Left:
    case ManeuverType::Right:
    case ManeuverType::Exit:
      return 2;
    case ManeuverType::SharpLeft:
    case ManeuverType::SharpRight:
      return 3;
    case ManeuverType::UTurnLeft:
    case ManeuverType::UTurnRight:
      return 4;
    default:
      return 0;
  }
}

// Order of trust. The phone's text is not first any more: every real Amap
// instruction ends with a generic verb ("...行驶168米左转"), so letting the text
// win means the geometry pass below -- the only thing that can tell a 30-degree
// bend from a 90-degree one -- never runs, and every movement collapses into
// left/right. Instead:
//   1. arrive / roundabout / u-turn are strong, unambiguous signals, text first;
//   2. the phone's own sharp/slight label is already finer than a bare left/right
//      and is kept;
//   3. geometry decides the shape inside the window;
//   4. the generic text verb is the fallback, then the phone's coarse label.
ManeuverType resolve_maneuver(const NavSnapshot& snapshot,
                              double distance_to_maneuver_m) {
  // "Arrive" marks the destination, not a manoeuvre. Amap attaches it to the
  // final step, whose offset can sit hundreds of metres before the endpoint,
  // so trusting it painted the flag while the distance number was still
  // counting down. The flag now waits until the core itself has confirmed the
  // arrival.
  const auto arrive_or_straight = [&snapshot](ManeuverType type) {
    if (type != ManeuverType::Arrive) return type;
    return snapshot.state == NavState::Arrived ? ManeuverType::Arrive
                                               : ManeuverType::Continue;
  };
  if (!snapshot.has_next_maneuver) {
    return arrive_or_straight(ManeuverType::Arrive);
  }
  const Maneuver& next = snapshot.next_maneuver;
  if (next.type == ManeuverType::Arrive) {
    return arrive_or_straight(next.type);
  }
  const ManeuverType from_text =
      maneuver_from_instruction(next.instruction, next.roundabout_exit);
  if (from_text == ManeuverType::Arrive) {
    return arrive_or_straight(from_text);
  }
  if (from_text == ManeuverType::Roundabout ||
      next.type == ManeuverType::Roundabout || next.roundabout_exit > 0) {
    return ManeuverType::Roundabout;
  }
  if (from_text == ManeuverType::UTurnLeft) {
    return ManeuverType::UTurnLeft;
  }
  if (next.type == ManeuverType::UTurnLeft ||
      next.type == ManeuverType::UTurnRight) {
    return next.type;
  }
  // The phone already named a partial turn; that is finer than a bare left or
  // right and beats whatever a sparse polyline suggests on the same side. But
  // the gateway files every "向左前方"/"靠左" instruction as slight, and that
  // wording also shows up on plain right-angle junctions, so the polyline may
  // sharpen the label when it reads the same side harder. It never softens one.
  if (next.type == ManeuverType::SlightLeft ||
      next.type == ManeuverType::SlightRight ||
      next.type == ManeuverType::SharpLeft ||
      next.type == ManeuverType::SharpRight) {
    const ManeuverType geometry =
        maneuver_from_geometry(snapshot, distance_to_maneuver_m);
    if (geometry != ManeuverType::Unknown &&
        geometry != ManeuverType::Continue &&
        ((is_left_hand(geometry) && is_left_hand(next.type)) ||
         (is_right_hand(geometry) && is_right_hand(next.type))) &&
        maneuver_rank(geometry) > maneuver_rank(next.type)) {
      return geometry;
    }
    return next.type;
  }
  const ManeuverType from_geometry =
      maneuver_from_geometry(snapshot, distance_to_maneuver_m);
  if (from_geometry != ManeuverType::Unknown &&
      from_geometry != ManeuverType::Continue) {
    // A reversal has no meaningful left/right side, so it is not compared
    // against the phone's label: a 175-degree bend is a u-turn whichever way it
    // curls.
    if (from_geometry == ManeuverType::UTurnLeft ||
        from_geometry == ManeuverType::UTurnRight) {
      return from_geometry;
    }
    if ((is_left_hand(from_geometry) && is_right_hand(next.type)) ||
        (is_right_hand(from_geometry) && is_left_hand(next.type))) {
      return next.type;
    }
    // A right angle thinned into 30-45 degree segments reads as a slight bend
    // and softens a plain left/right the phone already named. Re-read that
    // corner over a 100 m baseline before accepting the weaker glyph.
    if (maneuver_rank(next.type) >= 2 && maneuver_rank(from_geometry) < 2) {
      const ManeuverType wider = maneuver_from_geometry(
          snapshot, distance_to_maneuver_m, 100.0);
      if (wider != ManeuverType::Unknown && wider != ManeuverType::Continue &&
          ((is_left_hand(wider) && is_left_hand(next.type)) ||
           (is_right_hand(wider) && is_right_hand(next.type))) &&
          maneuver_rank(wider) >= maneuver_rank(next.type)) {
        return wider;
      }
    }
    return from_geometry;
  }
  if (from_text != ManeuverType::Unknown) {
    return from_text;
  }
  return next.type;
}

template <typename Integer>
Integer rounded_saturated(double value) {
  static_assert(std::numeric_limits<Integer>::is_integer,
                "destination must be an integer");
  if (!std::isfinite(value) || value <= 0.0) {
    return 0;
  }
  const double maximum =
      static_cast<double>(std::numeric_limits<Integer>::max());
  if (value >= maximum) {
    return std::numeric_limits<Integer>::max();
  }
  return static_cast<Integer>(std::floor(value + 0.5));
}

std::uint8_t progress_percent(const NavSnapshot& snapshot) {
  if (snapshot.state == NavState::Arrived) {
    return 100;
  }
  if (!std::isfinite(snapshot.total_distance_m) ||
      snapshot.total_distance_m <= 0.0 ||
      !std::isfinite(snapshot.route_progress_m)) {
    return 0;
  }
  const double percent = std::clamp(
      snapshot.route_progress_m / snapshot.total_distance_m * 100.0,
      0.0, 100.0);
  return rounded_saturated<std::uint8_t>(percent);
}

std::uint16_t normalized_heading(float value) {
  if (!std::isfinite(value)) {
    return 0;
  }
  double normalized = std::fmod(static_cast<double>(value), 360.0);
  if (normalized < 0.0) {
    normalized += 360.0;
  }
  return static_cast<std::uint16_t>(std::lround(normalized)) % 360;
}

std::uint32_t route_identity(const std::string& route_id) {
  if (route_id.empty()) return 0;
  std::uint32_t hash = 2'166'136'261U;
  for (const unsigned char byte : route_id) {
    hash ^= byte;
    hash *= 16'777'619U;
  }
  // Zero is reserved for the no-route state.
  return hash == 0 ? 1 : hash;
}

struct MapTransform {
  double origin_latitude_deg = 0.0;
  double origin_longitude_deg = 0.0;
  double longitude_pixels_per_degree = 1.0;
  float sin_heading = 0.0F;
  float cos_heading = 1.0F;
};

MapTransform map_transform(const NavSnapshot& snapshot) {
  const double finite_heading =
      std::isfinite(snapshot.heading_deg) ? snapshot.heading_deg : 0.0;
  const double heading = radians(finite_heading);
  return {
      snapshot.route_view_origin.latitude_deg,
      snapshot.route_view_origin.longitude_deg,
      std::cos(radians(snapshot.route_view_origin.latitude_deg)) *
          (kPi / 180.0 * kEarthRadiusM * kMapPixelsPerMeter),
      static_cast<float>(std::sin(heading)),
      static_cast<float>(std::cos(heading)),
  };
}

moto_ui_point_t project_map_point(const Gcj02Point& point,
                                  const MapTransform& transform) {
  // Subtract geographic doubles before converting to screen-space floats.
  // Casting lat/lon first would lose metres of precision; after subtraction,
  // float is precise well below one pixel and avoids software-double rotation
  // for hundreds of points on the ESP32-S3's single-precision FPU.
  const float east = static_cast<float>(
      (point.longitude_deg - transform.origin_longitude_deg) *
      transform.longitude_pixels_per_degree);
  const float north = static_cast<float>(
      (point.latitude_deg - transform.origin_latitude_deg) *
      (kPi / 180.0 * kEarthRadiusM * kMapPixelsPerMeter));

  // Heading-up transform: the rider remains fixed and every real street
  // translates/rotates together under it, as on an automotive minimap.
  const float right = east * transform.cos_heading -
                       north * transform.sin_heading;
  const float forward = east * transform.sin_heading +
                         north * transform.cos_heading;
  const float x = static_cast<float>(kVehicleX) + right;
  const float y = static_cast<float>(kVehicleY) - forward;
  return {
      static_cast<std::int16_t>(
          std::clamp(std::lround(x), -2'000L, 2'000L)),
      static_cast<std::int16_t>(
          std::clamp(std::lround(y), -2'000L, 2'000L)),
  };
}

void project_route_view(const NavSnapshot& snapshot,
                        const MapTransform& transform,
                        moto_ui_state_t& output) {
  output.route_point_count = 0;
  if (!snapshot.has_route_view || snapshot.route_view_point_count < 2) {
    return;
  }

  const std::size_t count = std::min<std::size_t>(
      snapshot.route_view_point_count, MOTO_UI_ROUTE_POINT_CAPACITY);
  for (std::size_t i = 0; i < count; ++i) {
    output.route_points[i] =
        project_map_point(snapshot.route_view_points[i], transform);
  }
  output.route_point_count = static_cast<std::uint8_t>(count);
}

void project_road_context(const NavSnapshot& snapshot,
                          const MapTransform& transform,
                          moto_ui_state_t& output) {
  output.road_point_count = 0;
  output.road_polyline_count = 0;
  if (!snapshot.has_route_view || !snapshot.has_road_context ||
      snapshot.road_context_point_count < 2 ||
      snapshot.road_context_polyline_count == 0) {
    return;
  }

  const std::size_t point_count = std::min<std::size_t>(
      snapshot.road_context_point_count, MOTO_UI_ROAD_POINT_CAPACITY);
  const std::size_t polyline_count = std::min<std::size_t>(
      snapshot.road_context_polyline_count,
      MOTO_UI_ROAD_POLYLINE_CAPACITY);
  for (std::size_t index = 0; index < point_count; ++index) {
    output.road_points[index] =
        project_map_point(snapshot.road_context_points[index], transform);
  }

  std::size_t accepted_polylines = 0;
  for (std::size_t index = 0; index < polyline_count; ++index) {
    const RoadContextPolylineSpan& span =
        snapshot.road_context_polylines[index];
    const std::size_t first = span.first_point_index;
    const std::size_t count = span.point_count;
    if (count < 2 || first >= point_count || count > point_count - first) {
      continue;
    }
    output.road_polylines[accepted_polylines] = {
        static_cast<std::uint8_t>(first),
        static_cast<std::uint8_t>(count),
        static_cast<std::uint8_t>(span.road_class),
    };
    ++accepted_polylines;
  }
  output.road_point_count = static_cast<std::uint8_t>(point_count);
  output.road_polyline_count =
      static_cast<std::uint8_t>(accepted_polylines);
}

void project_building_context(const NavSnapshot& snapshot,
                              const MapTransform& transform,
                              moto_ui_state_t& output) {
  output.building_point_count = 0;
  output.building_footprint_count = 0;
  if (!snapshot.has_route_view || !snapshot.has_building_context ||
      snapshot.building_context_point_count < 3 ||
      snapshot.building_context_footprint_count == 0) {
    return;
  }

  const std::size_t point_count = std::min<std::size_t>(
      snapshot.building_context_point_count,
      MOTO_UI_BUILDING_POINT_CAPACITY);
  const std::size_t footprint_count = std::min<std::size_t>(
      snapshot.building_context_footprint_count,
      MOTO_UI_BUILDING_FOOTPRINT_CAPACITY);
  for (std::size_t index = 0; index < point_count; ++index) {
    output.building_points[index] = project_map_point(
        snapshot.building_context_points[index], transform);
  }

  std::size_t accepted_footprints = 0;
  for (std::size_t index = 0; index < footprint_count; ++index) {
    const BuildingContextFootprintSpan& span =
        snapshot.building_context_footprints[index];
    const std::size_t first = span.first_point_index;
    const std::size_t count = span.point_count;
    if (count < 3 || first >= point_count || count > point_count - first) {
      continue;
    }
    output.building_footprints[accepted_footprints] = {
        static_cast<std::uint8_t>(first),
        static_cast<std::uint8_t>(count),
        static_cast<std::uint8_t>(span.building_class),
    };
    ++accepted_footprints;
  }
  output.building_point_count = static_cast<std::uint8_t>(point_count);
  output.building_footprint_count =
      static_cast<std::uint8_t>(accepted_footprints);
}

}  // namespace

NavPresenter::NavPresenter() { update(NavSnapshot{}); }

void NavPresenter::update(const NavSnapshot& snapshot) {
  road_name_.clear();
  next_road_name_.clear();
  if (snapshot.has_next_maneuver) {
    // NavSnapshot currently exposes the road associated with the next
    // maneuver, but no separate current-road field. Keep two owned copies so
    // both C pointers have an explicit, stable lifetime. A future current-road
    // field can replace road_name_ without changing moto_ui_state_t.
    road_name_ = snapshot.next_maneuver.road_name;
    next_road_name_ = snapshot.next_maneuver.road_name.empty()
                          ? snapshot.next_maneuver.instruction
                          : snapshot.next_maneuver.road_name;
  }

  ui_state_ = {};
  ui_state_.page = map_page(snapshot.display_page);
  ui_state_.mode = map_mode(snapshot);
  // The phone only distinguishes left from right, so the movement is resolved
  // from its readable instruction and the route geometry. Falling back on the
  // phone label keeps a real turn visible even when neither source is usable.
  ui_state_.maneuver = map_maneuver(resolve_maneuver(
      snapshot, snapshot.distance_to_next_maneuver_m));
  ui_state_.traffic = map_traffic(snapshot.traffic_ahead);
  ui_state_.distance_to_maneuver_m =
      rounded_saturated<std::uint32_t>(
          snapshot.distance_to_next_maneuver_m);
  ui_state_.remaining_distance_m =
      rounded_saturated<std::uint32_t>(snapshot.remaining_distance_m);
  ui_state_.remaining_time_s = snapshot.remaining_duration_s;
  ui_state_.route_progress_percent = progress_percent(snapshot);
  ui_state_.gps_accuracy_m =
      snapshot.has_usable_fix
          ? rounded_saturated<std::uint8_t>(snapshot.horizontal_accuracy_m)
          : 0;
  ui_state_.online = snapshot.network == NetworkState::Online ? 1 : 0;
  ui_state_.has_destination = snapshot.has_destination ? 1 : 0;
  ui_state_.route_request_in_flight =
      snapshot.route_request_in_flight ? 1 : 0;
  ui_state_.route_identity = route_identity(snapshot.route_id);
  ui_state_.route_generation = snapshot.route_generation;
  ui_state_.map_scene_revision = snapshot.map_scene_revision;
  ui_state_.speed_kph = rounded_saturated<std::uint16_t>(
      static_cast<double>(snapshot.speed_mps) * 3.6);
  ui_state_.speed_limit_kph = snapshot.speed_limit_kph;
  ui_state_.heading_deg = normalized_heading(snapshot.heading_deg);
  // Route, roads and buildings share one rider-centred transform. Besides
  // guaranteeing exact layer alignment, this avoids repeating trigonometry
  // hundreds of times on every high-rate IMU frame.
  const MapTransform transform = map_transform(snapshot);
  project_route_view(snapshot, transform, ui_state_);
  project_road_context(snapshot, transform, ui_state_);
  project_building_context(snapshot, transform, ui_state_);
  ui_state_.road_name = road_name_.empty() ? nullptr : road_name_.c_str();
  ui_state_.next_road_name =
      next_road_name_.empty() ? nullptr : next_road_name_.c_str();
}

const moto_ui_state_t& NavPresenter::ui_state() const { return ui_state_; }

void NavPresenter::apply_to_lvgl() const {
  moto_nav_ui_set_state(&ui_state_);
}

}  // namespace moto::nav

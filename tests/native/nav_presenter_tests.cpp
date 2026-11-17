#include "moto_nav_presenter.hpp"

#ifdef MOTO_NAV_UI_TEST_STUB
#include "moto_nav_ui_test_probe.hpp"
#endif

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>

namespace {

using namespace moto::nav;

int failures = 0;

#define CHECK(condition)                                                   \
  do {                                                                     \
    if (!(condition)) {                                                    \
      std::cerr << __FILE__ << ':' << __LINE__                             \
                << " CHECK failed: " #condition << '\n';                  \
      ++failures;                                                          \
    }                                                                      \
  } while (false)

void test_all_navigation_and_network_states() {
  struct StateCase {
    NavState input;
    moto_ui_mode_t expected;
  };
  constexpr std::array<StateCase, 6> cases{{
      {NavState::Idle, MOTO_UI_ACQUIRING_FIX},
      {NavState::Acquiring, MOTO_UI_ACQUIRING_FIX},
      {NavState::Planning, MOTO_UI_ACQUIRING_FIX},
      {NavState::Navigating, MOTO_UI_NAVIGATING},
      {NavState::Rerouting, MOTO_UI_REROUTING},
      {NavState::Arrived, MOTO_UI_ARRIVED},
  }};

  NavPresenter presenter;
  for (const StateCase& test : cases) {
    NavSnapshot snapshot;
    snapshot.state = test.input;
    snapshot.network = NetworkState::Online;
    presenter.update(snapshot);
    CHECK(presenter.ui_state().mode == test.expected);
    CHECK(presenter.ui_state().online == 1);
  }

  NavSnapshot snapshot;
  snapshot.state = NavState::Navigating;
  snapshot.network = NetworkState::Offline;
  presenter.update(snapshot);
  CHECK(presenter.ui_state().mode == MOTO_UI_OFFLINE);
  CHECK(presenter.ui_state().online == 0);

  snapshot.network = NetworkState::Connecting;
  presenter.update(snapshot);
  CHECK(presenter.ui_state().mode == MOTO_UI_NAVIGATING);
  CHECK(presenter.ui_state().online == 0);

  snapshot.state = NavState::Rerouting;
  snapshot.network = NetworkState::Offline;
  presenter.update(snapshot);
  CHECK(presenter.ui_state().mode == MOTO_UI_REROUTING);
  CHECK(presenter.ui_state().online == 0);

  snapshot.has_destination = true;
  snapshot.route_request_in_flight = true;
  presenter.update(snapshot);
  CHECK(presenter.ui_state().has_destination == 1);
  CHECK(presenter.ui_state().route_request_in_flight == 1);
}

void test_display_pages_map_without_changing_navigation_mode() {
  struct PageCase {
    DisplayPage input;
    moto_ui_page_t expected;
  };
  constexpr std::array<PageCase, 4> cases{{
      {DisplayPage::Navigation, MOTO_UI_PAGE_NAVIGATION},
      {DisplayPage::Speed, MOTO_UI_PAGE_SPEED},
      {DisplayPage::Compass, MOTO_UI_PAGE_COMPASS},
      {DisplayPage::Music, MOTO_UI_PAGE_MUSIC},
  }};
  NavPresenter presenter;
  NavSnapshot snapshot;
  snapshot.state = NavState::Rerouting;
  for(const PageCase& test : cases) {
    snapshot.display_page = test.input;
    presenter.update(snapshot);
    CHECK(presenter.ui_state().page == test.expected);
    CHECK(presenter.ui_state().mode == MOTO_UI_REROUTING);
  }
}

void test_all_maneuvers() {
  struct ManeuverCase {
    ManeuverType input;
    moto_maneuver_t expected;
  };
  constexpr std::array<ManeuverCase, 13> cases{{
      {ManeuverType::Unknown, MOTO_MANEUVER_STRAIGHT},
      {ManeuverType::Continue, MOTO_MANEUVER_STRAIGHT},
      {ManeuverType::SlightLeft, MOTO_MANEUVER_SLIGHT_LEFT},
      {ManeuverType::Left, MOTO_MANEUVER_LEFT},
      {ManeuverType::SharpLeft, MOTO_MANEUVER_LEFT},
      {ManeuverType::UTurnLeft, MOTO_MANEUVER_UTURN},
      {ManeuverType::SlightRight, MOTO_MANEUVER_SLIGHT_RIGHT},
      {ManeuverType::Right, MOTO_MANEUVER_RIGHT},
      {ManeuverType::SharpRight, MOTO_MANEUVER_RIGHT},
      {ManeuverType::UTurnRight, MOTO_MANEUVER_UTURN},
      {ManeuverType::Roundabout, MOTO_MANEUVER_ROUNDABOUT},
      {ManeuverType::Exit, MOTO_MANEUVER_RIGHT},
      // Arrive is a destination marker, not a turn: on a running route it stays
      // a straight arrow and the flag appears only after confirmed arrival (the
      // Arrived case below).
      {ManeuverType::Arrive, MOTO_MANEUVER_STRAIGHT},
  }};

  NavPresenter presenter;
  NavSnapshot snapshot;
  snapshot.state = NavState::Navigating;
  snapshot.has_next_maneuver = true;
  for (const ManeuverCase& test : cases) {
    snapshot.next_maneuver.type = test.input;
    presenter.update(snapshot);
    CHECK(presenter.ui_state().maneuver == test.expected);
  }

  snapshot.has_next_maneuver = false;
  snapshot.state = NavState::Arrived;
  presenter.update(snapshot);
  CHECK(presenter.ui_state().maneuver == MOTO_MANEUVER_ARRIVE);
}

void test_all_traffic_levels() {
  struct TrafficCase {
    TrafficLevel input;
    moto_traffic_t expected;
  };
  constexpr std::array<TrafficCase, 5> cases{{
      {TrafficLevel::Unknown, MOTO_TRAFFIC_UNKNOWN},
      {TrafficLevel::FreeFlow, MOTO_TRAFFIC_CLEAR},
      {TrafficLevel::Slow, MOTO_TRAFFIC_SLOW},
      {TrafficLevel::Congested, MOTO_TRAFFIC_CONGESTED},
      {TrafficLevel::Severe, MOTO_TRAFFIC_SEVERE},
  }};

  NavPresenter presenter;
  NavSnapshot snapshot;
  for (const TrafficCase& test : cases) {
    snapshot.traffic_ahead = test.input;
    presenter.update(snapshot);
    CHECK(presenter.ui_state().traffic == test.expected);
  }
}

void test_metrics_are_rounded_clamped_and_deterministic() {
  NavPresenter presenter;
  NavSnapshot snapshot;
  snapshot.route_id = "route-a";
  snapshot.state = NavState::Navigating;
  snapshot.has_usable_fix = true;
  snapshot.distance_to_next_maneuver_m = 12.6;
  snapshot.remaining_distance_m = 1'000.49;
  snapshot.remaining_duration_s = 721;
  snapshot.route_progress_m = 50.0;
  snapshot.total_distance_m = 199.0;
  snapshot.horizontal_accuracy_m = 3.6F;
  snapshot.speed_mps = 8.0F;
  snapshot.heading_deg = 359.6F;
  snapshot.speed_limit_kph = 70;
  presenter.update(snapshot);

  const moto_ui_state_t& state = presenter.ui_state();
  CHECK(state.distance_to_maneuver_m == 13);
  CHECK(state.remaining_distance_m == 1'000);
  CHECK(state.remaining_time_s == 721);
  CHECK(state.route_progress_percent == 25);
  CHECK(state.gps_accuracy_m == 4);
  CHECK(state.speed_kph == 29);
  CHECK(state.heading_deg == 0);
  CHECK(state.speed_limit_kph == 70);
  CHECK(state.route_identity != 0);
  const std::uint32_t first_route_identity = state.route_identity;

  snapshot.route_id = "route-b";
  presenter.update(snapshot);
  CHECK(presenter.ui_state().route_identity != first_route_identity);

  snapshot.distance_to_next_maneuver_m = -2.0;
  snapshot.remaining_distance_m =
      static_cast<double>(std::numeric_limits<std::uint32_t>::max()) +
      1'000.0;
  snapshot.route_progress_m = 250.0;
  snapshot.total_distance_m = 200.0;
  snapshot.horizontal_accuracy_m = 300.0F;
  presenter.update(snapshot);
  CHECK(presenter.ui_state().distance_to_maneuver_m == 0);
  CHECK(presenter.ui_state().remaining_distance_m ==
        std::numeric_limits<std::uint32_t>::max());
  CHECK(presenter.ui_state().route_progress_percent == 100);
  CHECK(presenter.ui_state().gps_accuracy_m == 255);

  snapshot.state = NavState::Arrived;
  snapshot.route_progress_m = 0.0;
  snapshot.total_distance_m = 0.0;
  presenter.update(snapshot);
  CHECK(presenter.ui_state().route_progress_percent == 100);
}

void test_heading_up_route_projection_keeps_rider_fixed() {
  NavPresenter presenter;
  NavSnapshot snapshot;
  snapshot.state = NavState::Navigating;
  snapshot.heading_deg = 90.0F;
  snapshot.distance_to_next_maneuver_m = 500.0;
  snapshot.has_route_view = true;
  snapshot.route_view_origin = {31.2304, 121.4737};
  snapshot.route_view_point_count = 2;
  snapshot.route_view_points[0] = snapshot.route_view_origin;
  snapshot.route_view_points[1] = {31.2304, 121.4747};
  presenter.update(snapshot);

  const moto_ui_state_t& state = presenter.ui_state();
  CHECK(state.route_point_count == 2);
  constexpr int rider_x = MOTO_UI_CANVAS_WIDTH / 2;
  constexpr int rider_y =
      (196 * MOTO_UI_CANVAS_HEIGHT + 180) / 360;
  CHECK(state.route_points[0].x == rider_x);
  CHECK(state.route_points[0].y == rider_y);
  // Facing east means an eastbound route is drawn straight ahead/up while
  // the vehicle marker itself remains at the fixed shared-view anchor.
  CHECK(std::abs(state.route_points[1].x - rider_x) <= 1);
  CHECK(state.route_points[1].y < state.route_points[0].y);
}

void test_non_finite_heading_keeps_map_projection_finite() {
  NavPresenter presenter;
  NavSnapshot snapshot;
  snapshot.state = NavState::Navigating;
  snapshot.heading_deg = std::numeric_limits<float>::quiet_NaN();
  snapshot.has_route_view = true;
  snapshot.route_view_origin = {31.2304, 121.4737};
  snapshot.route_view_point_count = 2;
  snapshot.route_view_points[0] = snapshot.route_view_origin;
  snapshot.route_view_points[1] = {31.2314, 121.4737};
  presenter.update(snapshot);

  const moto_ui_state_t& state = presenter.ui_state();
  CHECK(state.heading_deg == 0);
  CHECK(state.route_point_count == 2);
  constexpr int rider_x = MOTO_UI_CANVAS_WIDTH / 2;
  constexpr int rider_y =
      (196 * MOTO_UI_CANVAS_HEIGHT + 180) / 360;
  CHECK(state.route_points[0].x == rider_x);
  CHECK(state.route_points[0].y == rider_y);
}

void test_real_road_context_uses_the_same_heading_up_transform() {
  NavPresenter presenter;
  NavSnapshot snapshot;
  snapshot.state = NavState::Navigating;
  snapshot.heading_deg = 90.0F;
  snapshot.has_route_view = true;
  snapshot.route_view_origin = {36.67, 117.13};
  snapshot.route_view_point_count = 2;
  snapshot.route_view_points[0] = snapshot.route_view_origin;
  snapshot.route_view_points[1] = {36.67, 117.131};

  snapshot.has_road_context = true;
  snapshot.road_context_point_count = 4;
  snapshot.road_context_points[0] = snapshot.route_view_origin;
  snapshot.road_context_points[1] = {36.67, 117.131};
  snapshot.road_context_points[2] = {36.669, 117.13};
  snapshot.road_context_points[3] = {36.668, 117.13};
  snapshot.road_context_polyline_count = 2;
  snapshot.road_context_polylines[0] = {0, 2};
  snapshot.road_context_polylines[1] = {2, 2};
  presenter.update(snapshot);

  const moto_ui_state_t& state = presenter.ui_state();
  CHECK(state.road_point_count == 4);
  CHECK(state.road_polyline_count == 2);
  CHECK(state.road_polylines[0].first_point_index == 0);
  CHECK(state.road_polylines[0].point_count == 2);
  CHECK(state.road_polylines[1].first_point_index == 2);
  CHECK(state.road_polylines[1].point_count == 2);
  CHECK(state.road_points[0].x == state.route_points[0].x);
  CHECK(state.road_points[0].y == state.route_points[0].y);
  CHECK(state.road_points[1].x == state.route_points[1].x);
  CHECK(state.road_points[1].y == state.route_points[1].y);
}

void test_map_scene_classes_buildings_and_capacity_share_route_transform() {
  NavPresenter presenter;
  NavSnapshot snapshot;
  snapshot.state = NavState::Navigating;
  snapshot.heading_deg = 90.0F;
  snapshot.has_route_view = true;
  snapshot.route_view_origin = {36.67, 117.13};
  snapshot.route_view_point_count = 2;
  snapshot.route_view_points[0] = snapshot.route_view_origin;
  snapshot.route_view_points[1] = {36.67, 117.131};

  snapshot.has_road_context = true;
  snapshot.road_context_point_count = kRoadContextPointCapacity;
  snapshot.road_context_polyline_count = kRoadContextPolylineCapacity;
  for (std::size_t index = 0; index < kRoadContextPointCapacity; ++index) {
    snapshot.road_context_points[index] = {
        36.67 + static_cast<double>(index) * 0.000001,
        117.13 + static_cast<double>(index) * 0.000001,
    };
  }
  for (std::size_t index = 0; index < kRoadContextPolylineCapacity;
       ++index) {
    snapshot.road_context_polylines[index] = {
        static_cast<std::uint8_t>(index * 8), 8,
        index == 0 ? RoadContextClass::Motorway
                   : RoadContextClass::Service,
    };
  }

  snapshot.has_building_context = true;
  snapshot.building_context_point_count = kBuildingContextPointCapacity;
  snapshot.building_context_footprint_count =
      kBuildingContextFootprintCapacity;
  for (std::size_t index = 0; index < kBuildingContextPointCapacity;
       ++index) {
    snapshot.building_context_points[index] = {
        36.67 + static_cast<double>(index) * 0.000001,
        117.13 + static_cast<double>(index) * 0.000002,
    };
  }
  for (std::size_t index = 0;
       index < kBuildingContextFootprintCapacity; ++index) {
    snapshot.building_context_footprints[index] = {
        static_cast<std::uint8_t>(index * 8), 8,
        index == 0 ? BuildingContextClass::Landmark
                   : BuildingContextClass::Generic,
    };
  }

  presenter.update(snapshot);
  const moto_ui_state_t& state = presenter.ui_state();
  CHECK(state.road_point_count == MOTO_UI_ROAD_POINT_CAPACITY);
  CHECK(state.road_polyline_count == MOTO_UI_ROAD_POLYLINE_CAPACITY);
  CHECK(state.road_polylines[0].road_class ==
        static_cast<std::uint8_t>(RoadContextClass::Motorway));
  CHECK(state.road_polylines[1].road_class ==
        static_cast<std::uint8_t>(RoadContextClass::Service));
  CHECK(state.building_point_count == MOTO_UI_BUILDING_POINT_CAPACITY);
  CHECK(state.building_footprint_count ==
        MOTO_UI_BUILDING_FOOTPRINT_CAPACITY);
  CHECK(state.building_footprints[0].building_class ==
        static_cast<std::uint8_t>(BuildingContextClass::Landmark));

  // The first point of every layer is the rider origin, proving route, road
  // and building coordinates share one heading-up projection.
  CHECK(state.road_points[0].x == state.route_points[0].x);
  CHECK(state.road_points[0].y == state.route_points[0].y);
  CHECK(state.building_points[0].x == state.route_points[0].x);
  CHECK(state.building_points[0].y == state.route_points[0].y);
}

void test_map_scale_does_not_jump_at_maneuver_distance_thresholds() {
  NavPresenter presenter;
  NavSnapshot snapshot;
  snapshot.state = NavState::Navigating;
  snapshot.heading_deg = 0.0F;
  snapshot.has_route_view = true;
  snapshot.route_view_origin = {36.67, 117.13};
  snapshot.route_view_point_count = 2;
  snapshot.route_view_points[0] = snapshot.route_view_origin;
  snapshot.route_view_points[1] = {36.671, 117.13};
  snapshot.has_road_context = true;
  snapshot.road_context_point_count = 2;
  snapshot.road_context_points[0] = snapshot.route_view_origin;
  snapshot.road_context_points[1] = {36.671, 117.13};
  snapshot.road_context_polyline_count = 1;
  snapshot.road_context_polylines[0] = {0, 2};

  constexpr std::array<double, 8> distances{{
      89.9, 90.0, 90.1, 259.9, 260.0, 260.1, 699.9, 700.1,
  }};
  moto_ui_point_t expected_route{};
  moto_ui_point_t expected_road{};
  bool have_expected = false;
  for (double distance : distances) {
    snapshot.distance_to_next_maneuver_m = distance;
    presenter.update(snapshot);
    const moto_ui_state_t& state = presenter.ui_state();
    CHECK(state.route_point_count == 2);
    CHECK(state.road_point_count == 2);
    if (!have_expected) {
      expected_route = state.route_points[1];
      expected_road = state.road_points[1];
      have_expected = true;
    } else {
      CHECK(state.route_points[1].x == expected_route.x);
      CHECK(state.route_points[1].y == expected_route.y);
      CHECK(state.road_points[1].x == expected_road.x);
      CHECK(state.road_points[1].y == expected_road.y);
    }
  }
}

void test_missing_or_incomplete_geometry_stays_empty() {
  NavPresenter presenter;
  NavSnapshot snapshot;
  snapshot.state = NavState::Navigating;
  snapshot.network = NetworkState::Online;
  snapshot.distance_to_next_maneuver_m = 300.0;

  // Navigation metadata is not route geometry. The formal display must wait
  // for phone-provided points instead of inventing a decorative route.
  snapshot.has_route_view = false;
  snapshot.route_view_point_count = 0;
  presenter.update(snapshot);
  CHECK(presenter.ui_state().route_point_count == 0);

  snapshot.has_route_view = true;
  snapshot.route_view_point_count = 1;
  snapshot.route_view_points[0] = {31.2304, 121.4737};
  presenter.update(snapshot);
  CHECK(presenter.ui_state().route_point_count == 0);
}

void test_presenter_owns_road_name_storage() {
  NavPresenter presenter;
  NavSnapshot snapshot;
  snapshot.has_next_maneuver = true;
  snapshot.next_maneuver.type = ManeuverType::Right;
  snapshot.next_maneuver.road_name = "中山路";
  snapshot.next_maneuver.instruction = "右转";
  presenter.update(snapshot);

  snapshot.next_maneuver.road_name.assign(2'048, 'x');
  snapshot.next_maneuver.instruction.clear();
  CHECK(presenter.ui_state().road_name != nullptr);
  CHECK(presenter.ui_state().next_road_name != nullptr);
  CHECK(std::strcmp(presenter.ui_state().road_name, "中山路") == 0);
  CHECK(std::strcmp(presenter.ui_state().next_road_name, "中山路") == 0);

  snapshot.next_maneuver.road_name.clear();
  snapshot.next_maneuver.instruction = "靠左行驶";
  presenter.update(snapshot);
  CHECK(presenter.ui_state().road_name == nullptr);
  CHECK(presenter.ui_state().next_road_name != nullptr);
  CHECK(std::strcmp(presenter.ui_state().next_road_name, "靠左行驶") == 0);

  snapshot.has_next_maneuver = false;
  presenter.update(snapshot);
  CHECK(presenter.ui_state().road_name == nullptr);
  CHECK(presenter.ui_state().next_road_name == nullptr);
}

// The phone labels every movement as left or right only, so the sharper glyphs
// are inferred on the device. These cases pin the priority order: readable
// instruction first, then route geometry, then the phone's own label.
void test_maneuver_inference_from_instruction() {
  struct TextCase {
    const char* instruction;
    ManeuverType phone_type;
    moto_maneuver_t expected;
  };
  constexpr std::array<TextCase, 8> cases{{
      {"在环岛第2出口驶出", ManeuverType::Right, MOTO_MANEUVER_ROUNDABOUT},
      {"前方掉头", ManeuverType::Left, MOTO_MANEUVER_UTURN},
      {"继续直行 800 米", ManeuverType::Right, MOTO_MANEUVER_STRAIGHT},
      {"稍向右转", ManeuverType::Right, MOTO_MANEUVER_SLIGHT_RIGHT},
      {"稍向左转", ManeuverType::Left, MOTO_MANEUVER_SLIGHT_LEFT},
      {"右转进入人民路", ManeuverType::Left, MOTO_MANEUVER_RIGHT},
      {"向左前方行驶", ManeuverType::Left, MOTO_MANEUVER_SLIGHT_LEFT},
      // "Arrive" is a destination marker, not guidance: while the route is
      // still running the same final instruction must stay a straight arrow.
      {"到达目的地", ManeuverType::Right, MOTO_MANEUVER_STRAIGHT},
  }};

  NavPresenter presenter;
  for (const TextCase& test : cases) {
    NavSnapshot snapshot;
    snapshot.state = NavState::Navigating;
    snapshot.network = NetworkState::Online;
    snapshot.has_next_maneuver = true;
    snapshot.next_maneuver.type = test.phone_type;
    snapshot.next_maneuver.instruction = test.instruction;
    snapshot.distance_to_next_maneuver_m = 120.0;
    presenter.update(snapshot);
    CHECK(presenter.ui_state().maneuver == test.expected);
  }

  // A roundabout exit number is a fact from the phone, not a guess.
  NavSnapshot snapshot;
  snapshot.state = NavState::Navigating;
  snapshot.has_next_maneuver = true;
  snapshot.next_maneuver.type = ManeuverType::Right;
  snapshot.next_maneuver.roundabout_exit = 2;
  presenter.update(snapshot);
  CHECK(presenter.ui_state().maneuver == MOTO_MANEUVER_ROUNDABOUT);

  // Once the core confirms the arrival the very same step paints the flag.
  NavSnapshot arrived;
  arrived.state = NavState::Arrived;
  arrived.has_next_maneuver = true;
  arrived.next_maneuver.type = ManeuverType::Right;
  arrived.next_maneuver.instruction = "到达目的地";
  presenter.update(arrived);
  CHECK(presenter.ui_state().maneuver == MOTO_MANEUVER_ARRIVE);
}

// A rider-centred window: five points north, then three points along `turn_deg`
// (clockwise from north). The corner sits 160 m ahead, 40 m per segment.
void build_bend_route(NavSnapshot* snapshot, double turn_deg) {
  constexpr int kNorthPoints = 5;
  constexpr double kStepM = 40.0;
  constexpr double kLatDegM = 111'195.0;
  const double lat0 = 36.6;
  const double lon0 = 117.0;
  const double m_per_deg_lon =
      kLatDegM * std::cos(lat0 * 3.14159265358979323846 / 180.0);

  snapshot->has_route_view = true;
  snapshot->route_view_origin = {lat0, lon0};
  snapshot->route_view_point_count =
      static_cast<std::uint8_t>(kNorthPoints + 3);
  double lat = lat0;
  double lon = lon0;
  for (int index = 0; index < kNorthPoints; ++index) {
    snapshot->route_view_points[static_cast<std::size_t>(index)] = {lat, lon};
    lat += kStepM / kLatDegM;
  }
  const double radians = turn_deg * 3.14159265358979323846 / 180.0;
  // Restart from the corner itself so the direction change lands exactly on the
  // vertex the presenter looks for.
  lat = lat0 + (kNorthPoints - 1) * kStepM / kLatDegM;
  lon = lon0;
  for (int index = 0; index < 3; ++index) {
    snapshot->route_view_points[static_cast<std::size_t>(kNorthPoints + index)] =
        {lat, lon};
    lat += kStepM * std::cos(radians) / kLatDegM;
    lon += kStepM * std::sin(radians) / m_per_deg_lon;
  }
}

// The same right-angle corner, but chamfered: five points north, then 40 m legs
// at 30, 60 and 90 degrees. A single leg reads 30 degrees, the whole corner is
// 90.
void build_chamfered_right_turn(NavSnapshot* snapshot) {
  constexpr int kNorthPoints = 5;
  constexpr double kStepM = 40.0;
  constexpr double kLatDegM = 111'195.0;
  constexpr std::array<double, 4> kLegDeg{30.0, 60.0, 90.0, 90.0};
  const double lat0 = 36.6;
  const double lon0 = 117.0;
  const double m_per_deg_lon =
      kLatDegM * std::cos(lat0 * 3.14159265358979323846 / 180.0);

  snapshot->has_route_view = true;
  snapshot->route_view_origin = {lat0, lon0};
  snapshot->route_view_point_count =
      static_cast<std::uint8_t>(kNorthPoints + kLegDeg.size());
  double lat = lat0;
  double lon = lon0;
  for (int index = 0; index < kNorthPoints; ++index) {
    snapshot->route_view_points[static_cast<std::size_t>(index)] = {lat, lon};
    lat += kStepM / kLatDegM;
  }
  lat = lat0 + (kNorthPoints - 1) * kStepM / kLatDegM;
  lon = lon0;
  for (std::size_t index = 0; index < kLegDeg.size(); ++index) {
    snapshot->route_view_points[static_cast<std::size_t>(kNorthPoints) + index] =
        {lat, lon};
    const double radians = kLegDeg[index] * 3.14159265358979323846 / 180.0;
    lat += kStepM * std::cos(radians) / kLatDegM;
    lon += kStepM * std::sin(radians) / m_per_deg_lon;
  }
}

void test_maneuver_inference_from_geometry() {
  struct BendCase {
    double turn_deg;
    ManeuverType phone_type;
    moto_maneuver_t expected;
  };
  constexpr std::array<BendCase, 5> cases{{
      {30.0, ManeuverType::Right, MOTO_MANEUVER_SLIGHT_RIGHT},
      {-30.0, ManeuverType::Left, MOTO_MANEUVER_SLIGHT_LEFT},
      {90.0, ManeuverType::Right, MOTO_MANEUVER_RIGHT},
      {-90.0, ManeuverType::Left, MOTO_MANEUVER_LEFT},
      {175.0, ManeuverType::Left, MOTO_MANEUVER_UTURN},
  }};

  NavPresenter presenter;
  for (const BendCase& test : cases) {
    NavSnapshot snapshot;
    snapshot.state = NavState::Navigating;
    snapshot.network = NetworkState::Online;
    snapshot.has_next_maneuver = true;
    snapshot.next_maneuver.type = test.phone_type;
    snapshot.distance_to_next_maneuver_m = 160.0;
    build_bend_route(&snapshot, test.turn_deg);
    presenter.update(snapshot);
    CHECK(presenter.ui_state().maneuver == test.expected);
  }

  // A bend that points the other way never overrides the phone: a left-hand
  // phone label plus a right-hand bend stays left.
  NavSnapshot snapshot;
  snapshot.state = NavState::Navigating;
  snapshot.has_next_maneuver = true;
  snapshot.next_maneuver.type = ManeuverType::Left;
  snapshot.distance_to_next_maneuver_m = 160.0;
  build_bend_route(&snapshot, 30.0);
  presenter.update(snapshot);
  CHECK(presenter.ui_state().maneuver == MOTO_MANEUVER_LEFT);

  // The corner is far outside the short view window: keep the phone label
  // instead of inventing guidance from a vertex that is not there.
  NavSnapshot far_snapshot;
  far_snapshot.state = NavState::Navigating;
  far_snapshot.has_next_maneuver = true;
  far_snapshot.next_maneuver.type = ManeuverType::Right;
  far_snapshot.distance_to_next_maneuver_m = 900.0;
  build_bend_route(&far_snapshot, 90.0);
  presenter.update(far_snapshot);
  CHECK(presenter.ui_state().maneuver == MOTO_MANEUVER_RIGHT);

  // A chamfered right angle: one leg reads 30 degrees, so it used to be shown as
  // "keep right". Measured over the corner it is a full right turn.
  NavSnapshot chamfered;
  chamfered.state = NavState::Navigating;
  chamfered.network = NetworkState::Online;
  chamfered.has_next_maneuver = true;
  chamfered.next_maneuver.type = ManeuverType::Right;
  chamfered.distance_to_next_maneuver_m = 160.0;
  build_chamfered_right_turn(&chamfered);
  presenter.update(chamfered);
  CHECK(presenter.ui_state().maneuver == MOTO_MANEUVER_RIGHT);
}

// Real Amap instructions always end with a generic verb ("...行驶168米左转"),
// so a text-first rule would collapse every movement into left/right and the
// geometry pass would never run. With a left label plus a 30-degree left bend
// the screen must show the slight glyph, not the full left arrow.
void test_generic_instruction_verb_stays_subordinate() {
  NavPresenter presenter;
  NavSnapshot snapshot;
  snapshot.state = NavState::Navigating;
  snapshot.network = NetworkState::Online;
  snapshot.has_next_maneuver = true;
  snapshot.next_maneuver.type = ManeuverType::Left;
  // 向东南行驶168米左转
  snapshot.next_maneuver.instruction =
      "\xE5\x90\x91\xE4\xB8\x9C\xE5\x8D\x97\xE8\xA1\x8C\xE9\xA9\xB6\x31\x36\x38"
      "\xE7\xB1\xB3\xE5\xB7\xA6\xE8\xBD\xAC";
  snapshot.distance_to_next_maneuver_m = 160.0;
  build_bend_route(&snapshot, -30.0);
  presenter.update(snapshot);
  CHECK(presenter.ui_state().maneuver == MOTO_MANEUVER_SLIGHT_LEFT);

  // Right label + 40-degree right bend: the slight right glyph, even though the
  // text also ends in 右转.
  NavSnapshot right_snapshot;
  right_snapshot.state = NavState::Navigating;
  right_snapshot.network = NetworkState::Online;
  right_snapshot.has_next_maneuver = true;
  right_snapshot.next_maneuver.type = ManeuverType::Right;
  right_snapshot.next_maneuver.instruction =
      "\x34\x30\x30\xE7\xB1\xB3\xE5\x8F\xB3\xE8\xBD\xAC";  // 400米右转
  right_snapshot.distance_to_next_maneuver_m = 160.0;
  build_bend_route(&right_snapshot, 40.0);
  presenter.update(right_snapshot);
  CHECK(presenter.ui_state().maneuver == MOTO_MANEUVER_SLIGHT_RIGHT);
}

#ifdef MOTO_NAV_UI_TEST_STUB
void test_apply_to_lvgl_uses_last_owned_state() {
  moto::test::reset_nav_ui_probe();
  NavPresenter presenter;
  NavSnapshot snapshot;
  snapshot.state = NavState::Navigating;
  snapshot.has_next_maneuver = true;
  snapshot.next_maneuver.type = ManeuverType::Left;
  snapshot.next_maneuver.road_name = "环城北路";
  presenter.update(snapshot);
  presenter.apply_to_lvgl();

  CHECK(moto::test::nav_ui_apply_count() == 1);
  const moto_ui_state_t& applied =
      moto::test::last_applied_nav_ui_state();
  CHECK(applied.maneuver == MOTO_MANEUVER_LEFT);
  CHECK(applied.next_road_name != nullptr);
  CHECK(std::strcmp(applied.next_road_name, "环城北路") == 0);
}
#endif

}  // namespace

int main() {
  test_all_navigation_and_network_states();
  test_display_pages_map_without_changing_navigation_mode();
  test_all_maneuvers();
  test_all_traffic_levels();
  test_metrics_are_rounded_clamped_and_deterministic();
  test_heading_up_route_projection_keeps_rider_fixed();
  test_non_finite_heading_keeps_map_projection_finite();
  test_real_road_context_uses_the_same_heading_up_transform();
  test_map_scene_classes_buildings_and_capacity_share_route_transform();
  test_map_scale_does_not_jump_at_maneuver_distance_thresholds();
  test_missing_or_incomplete_geometry_stays_empty();
  test_presenter_owns_road_name_storage();
  test_maneuver_inference_from_instruction();
  test_maneuver_inference_from_geometry();
  test_generic_instruction_verb_stays_subordinate();
#ifdef MOTO_NAV_UI_TEST_STUB
  test_apply_to_lvgl_uses_last_owned_state();
#endif

  if (failures != 0) {
    std::cerr << failures << " nav_presenter checks failed\n";
    return EXIT_FAILURE;
  }
  std::cout << "All nav_presenter checks passed\n";
  return EXIT_SUCCESS;
}

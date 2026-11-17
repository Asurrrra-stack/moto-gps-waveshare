// Local verification helper (not part of the upstream test suite): renders the
// real moto_nav_ui offscreen and dumps raw RGB565 frames so the shadow effects,
// icon joints and map styling can be judged before flashing a board.

#include "moto_nav_ui.h"

#include <lvgl.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

constexpr int kWidth = MOTO_UI_CANVAS_WIDTH;
constexpr int kHeight = MOTO_UI_CANVAS_HEIGHT;
constexpr int kBytesPerPixel = LV_COLOR_DEPTH / 8;

std::vector<std::uint8_t> frame;

void flush_cb(lv_display_t* display, const lv_area_t* area,
              std::uint8_t* px_map) {
  for (int y = area->y1; y <= area->y2; ++y) {
    const std::size_t row_bytes =
        static_cast<std::size_t>(area->x2 - area->x1 + 1) * kBytesPerPixel;
    const std::size_t src = static_cast<std::size_t>(y - area->y1) *
                            static_cast<std::size_t>(area->x2 - area->x1 + 1) *
                            kBytesPerPixel;
    const std::size_t dst =
        (static_cast<std::size_t>(y) * kWidth +
         static_cast<std::size_t>(area->x1)) * kBytesPerPixel;
    std::memcpy(frame.data() + dst, px_map + src, row_bytes);
  }
  lv_display_flush_ready(display);
}

void pump(int cycles = 40) {
  for (int i = 0; i < cycles; ++i) {
    lv_tick_inc(25);
    lv_timer_handler();
  }
}

int counters = 0;

void dump(const std::string& name, const std::string& dir, int cycles = 40) {
  pump(cycles);
  const std::string path = dir + "/" + name + ".raw";
  FILE* file = std::fopen(path.c_str(), "wb");
  std::fwrite(frame.data(), 1, frame.size(), file);
  std::fclose(file);
  ++counters;
}

// A grid-city window built from scratch: four horizontal streets, eight
// vertical ones and the blocks they enclose. It is the same kind of simplified
// MapScene the phone sends, dense enough to judge what a real street net looks
// like under the route and the fade band.
moto_ui_state_t base_state() {
  moto_ui_state_t state{};
  state.page = MOTO_UI_PAGE_NAVIGATION;
  state.mode = MOTO_UI_NAVIGATING;
  state.maneuver = MOTO_MANEUVER_LEFT;
  state.traffic = MOTO_TRAFFIC_CLEAR;
  state.distance_to_maneuver_m = 220;
  state.remaining_distance_m = 4'200;
  state.remaining_time_s = 600;
  state.route_progress_percent = 35;
  state.gps_accuracy_m = 4;
  state.online = 1;
  state.has_destination = 1;
  state.route_identity = 12;
  state.route_generation = 2;
  state.map_scene_revision = 3;
  state.speed_kph = 62;
  state.speed_limit_kph = 60;
  state.heading_deg = 95;
  // The preview build only carries Montserrat, so street names stay empty: a
  // CJK string here would render as tofu and misrepresent the panel.
  state.road_name = nullptr;
  state.next_road_name = nullptr;

  // Route: it runs up the street the marker sits on, then bends right along the
  // top street, so the white line follows the street net instead of ignoring it.
  state.route_point_count = 12;
  const std::int16_t route_x[12] = {214, 214, 214, 214, 215, 219,
                                    230, 250, 278, 310, 345, 380};
  const std::int16_t route_y[12] = {290, 262, 232, 204, 176, 148,
                                    124, 104, 90, 82, 79, 76};
  for (int i = 0; i < 12; ++i) state.route_points[i] = {route_x[i], route_y[i]};

  std::uint8_t points = 0;
  std::uint8_t polylines = 0;
  auto add_road = [&](std::int16_t x0, std::int16_t y0, std::int16_t x1,
                      std::int16_t y1, std::uint8_t road_class) {
    state.road_points[points] = {x0, y0};
    state.road_points[points + 1] = {x1, y1};
    state.road_polylines[polylines] = {points, 2, road_class};
    points = static_cast<std::uint8_t>(points + 2);
    polylines = static_cast<std::uint8_t>(polylines + 1);
  };

  // A handful of streets crossing at shallow angles, the way a real tile looks.
  // The earlier fixture was a perfectly orthogonal, evenly spaced grid, which
  // read as a spreadsheet and suggested the panel was drawing a table: it was
  // not, that grid was simply fixture scenery. On the device these polylines
  // come from the phone's map tiles, so they are skewed, sparse and irregular.
  add_road(0, 96, 466, 154, 2);     // long street drifting down to the right
  add_road(188, 0, 292, 300, 3);    // diagonal, top to bottom
  add_road(258, 0, 168, 300, 1);    // diagonal the other way
  add_road(0, 248, 466, 198, 4);    // lower street, rising to the right
  state.road_point_count = points;
  state.road_polyline_count = polylines;

  std::uint8_t building_points = 0;
  std::uint8_t footprints = 0;
  auto add_block = [&](std::int16_t x0, std::int16_t y0, std::int16_t x1,
                       std::int16_t y1, std::uint8_t building_class) {
    state.building_points[building_points + 0] = {x0, y0};
    state.building_points[building_points + 1] = {x1, y0};
    state.building_points[building_points + 2] = {x1, y1};
    state.building_points[building_points + 3] = {x0, y1};
    state.building_footprints[footprints] = {building_points, 4, building_class};
    building_points = static_cast<std::uint8_t>(building_points + 4);
    footprints = static_cast<std::uint8_t>(footprints + 1);
  };

  // Real footprints arrive as irregular polygons; a few quiet rectangular
  // outlines are enough to check the tone, the stroke and the landmark pair.
  add_block(48, 22, 132, 74, 0);
  add_block(306, 38, 378, 92, 1);
  add_block(96, 168, 178, 218, 0);
  state.building_point_count = building_points;
  state.building_footprint_count = footprints;
  return state;
}

}  // namespace

int main(int argc, char** argv) {
  const std::string dir = argc > 1 ? argv[1] : ".";

  lv_init();
  static std::vector<std::uint8_t> draw_buffer(
      (static_cast<std::size_t>(kWidth) * 466U * kBytesPerPixel));
  frame.assign(static_cast<std::size_t>(kWidth) * kHeight * kBytesPerPixel, 0);
  lv_display_t* display = lv_display_create(kWidth, kHeight);
  lv_display_set_flush_cb(display, flush_cb);
  lv_display_set_buffers(display, draw_buffer.data(), nullptr,
                          draw_buffer.size(), LV_DISPLAY_RENDER_MODE_PARTIAL);

  // Power-on wordmark, drawn before the full UI exists. The mark fades in over
  // 230 ms after a 30 ms delay, holds until 910 ms, then fades back out, so
  // advance to 500 ms and dump without extra pumping: one second more would
  // land after the fade and capture a black frame.
  moto_nav_ui_show_boot_screen();
  pump(20);
  dump("01_boot", dir, 0);
  pump(240);

  moto_nav_ui_create();
  moto_nav_ui_set_reduce_motion(1);
  moto_nav_ui_set_music_page_enabled(1);

  moto_battery_state_t battery{};
  battery.present = 1;
  battery.charging = 0;
  battery.percent = 78;
  battery.millivolts = 3'912;
  moto_nav_ui_set_battery(&battery);

  moto_ui_state_t state = base_state();
  moto_nav_ui_set_phone_connection(MOTO_UI_PHONE_ONLINE);

  // --- Navigation page, one frame per maneuver --------------------------
  state.page = MOTO_UI_PAGE_NAVIGATION;
  state.mode = MOTO_UI_NAVIGATING;
  state.online = 1;
  state.traffic = MOTO_TRAFFIC_CLEAR;
  state.speed_kph = 62;
  state.maneuver = MOTO_MANEUVER_LEFT;
  state.distance_to_maneuver_m = 220;
  state.speed_limit_kph = 60;
  moto_nav_ui_set_state(&state);
  dump("02_navigation_left", dir);

  state.maneuver = MOTO_MANEUVER_RIGHT;
  state.distance_to_maneuver_m = 240;
  state.traffic = MOTO_TRAFFIC_SLOW;
  state.speed_limit_kph = 80;
  moto_nav_ui_set_state(&state);
  dump("03_navigation_right", dir);

  state.maneuver = MOTO_MANEUVER_ROUNDABOUT;
  state.distance_to_maneuver_m = 190;
  state.traffic = MOTO_TRAFFIC_CLEAR;
  state.speed_limit_kph = 40;
  moto_nav_ui_set_state(&state);
  dump("04_navigation_roundabout", dir);

  state.maneuver = MOTO_MANEUVER_UTURN;
  state.distance_to_maneuver_m = 210;
  moto_nav_ui_set_state(&state);
  dump("05_navigation_uturn", dir);

  state.maneuver = MOTO_MANEUVER_SLIGHT_RIGHT;
  state.distance_to_maneuver_m = 260;
  state.speed_limit_kph = 50;
  moto_nav_ui_set_state(&state);
  dump("06_navigation_slight_right", dir);

  state.maneuver = MOTO_MANEUVER_SLIGHT_LEFT;
  state.distance_to_maneuver_m = 300;
  moto_nav_ui_set_state(&state);
  dump("07_navigation_slight_left", dir);

  state.maneuver = MOTO_MANEUVER_STRAIGHT;
  state.distance_to_maneuver_m = 520;
  state.speed_limit_kph = 70;
  moto_nav_ui_set_state(&state);
  dump("08_navigation_straight", dir);

  // Beyond the reveal threshold: the arrow falls back to straight and the
  // distance switches to kilometres.
  state.maneuver = MOTO_MANEUVER_LEFT;
  state.distance_to_maneuver_m = 1'250;
  state.route_progress_percent = 68;
  moto_nav_ui_set_state(&state);
  dump("09_navigation_far_km", dir);

  // Congested: the progress arc is the only coloured element on this page.
  state.distance_to_maneuver_m = 340;
  state.traffic = MOTO_TRAFFIC_CONGESTED;
  state.route_progress_percent = 82;
  moto_nav_ui_set_state(&state);
  dump("10_navigation_congested", dir);

  state.traffic = MOTO_TRAFFIC_CLEAR;
  state.route_progress_percent = 35;
  state.mode = MOTO_UI_REROUTING;
  state.distance_to_maneuver_m = 150;
  moto_nav_ui_set_state(&state);
  dump("11_navigation_rerouting", dir);

  state.mode = MOTO_UI_ARRIVED;
  state.maneuver = MOTO_MANEUVER_ARRIVE;
  state.distance_to_maneuver_m = 0;
  state.speed_limit_kph = 0;
  state.route_progress_percent = 100;
  moto_nav_ui_set_state(&state);
  dump("12_navigation_arrived", dir);

  // Charging pack: the header chip turns green without touching the map.
  state.mode = MOTO_UI_NAVIGATING;
  state.maneuver = MOTO_MANEUVER_LEFT;
  state.distance_to_maneuver_m = 180;
  state.speed_limit_kph = 60;
  state.route_progress_percent = 35;
  battery.charging = 1;
  battery.percent = 46;
  moto_nav_ui_set_battery(&battery);
  moto_nav_ui_set_state(&state);
  dump("13_navigation_charging", dir);

  // --- Power page -------------------------------------------------------
  battery.charging = 0;
  battery.percent = 78;
  battery.millivolts = 3'912;
  moto_nav_ui_set_battery(&battery);
  moto_nav_ui_set_page(MOTO_UI_PAGE_BATTERY);
  dump("14_battery_78", dir);

  battery.percent = 9;
  battery.millivolts = 3'320;
  moto_nav_ui_set_battery(&battery);
  dump("15_battery_low", dir);

  battery.charging = 1;
  battery.percent = 46;
  battery.millivolts = 3'760;
  moto_nav_ui_set_battery(&battery);
  dump("16_battery_charging", dir);

  battery.present = 0;
  battery.charging = 0;
  battery.percent = 0;
  battery.millivolts = 0;
  moto_nav_ui_set_battery(&battery);
  dump("17_battery_usb", dir);

  // --- Speed page -------------------------------------------------------
  battery.present = 1;
  battery.percent = 78;
  battery.millivolts = 3'912;
  moto_nav_ui_set_battery(&battery);
  state.page = MOTO_UI_PAGE_SPEED;
  // The POWER page pins itself with battery_override, so set_state alone cannot
  // pull the instrument away from it; leave the page explicitly first.
  moto_nav_ui_set_page(MOTO_UI_PAGE_SPEED);
  state.speed_kph = 72;
  moto_nav_ui_set_state(&state);
  dump("18_speed", dir);

  // --- Compass page -----------------------------------------------------
  state.page = MOTO_UI_PAGE_COMPASS;
  moto_nav_ui_set_page(MOTO_UI_PAGE_COMPASS);
  state.heading_deg = 95;
  state.speed_kph = 72;
  state.gps_accuracy_m = 4;
  moto_nav_ui_set_state(&state);
  dump("19_compass", dir);

  // --- Music page -------------------------------------------------------
  moto_music_state_t music{};
  music.source_name = "APPLE MUSIC";
  music.track_title = "NIGHT RIDE";
  music.artist_name = "PHONE NOW PLAYING";
  music.connected = 1;
  music.playing = 1;
  music.like_available = 1;
  music.liked = 0;
  state.page = MOTO_UI_PAGE_MUSIC;
  moto_nav_ui_set_page(MOTO_UI_PAGE_MUSIC);
  moto_nav_ui_set_state(&state);
  moto_nav_ui_set_music_state(&music);
  dump("20_music_playing", dir);

  music.playing = 0;
  moto_nav_ui_set_music_state(&music);
  dump("21_music_paused", dir);

  music.track_title = nullptr;
  music.artist_name = nullptr;
  music.connected = 0;
  music.like_available = 0;
  moto_nav_ui_set_music_state(&music);
  dump("22_music_idle", dir);

  // --- Pairing lifecycle on the navigation page ------------------------
  state.page = MOTO_UI_PAGE_NAVIGATION;
  moto_nav_ui_set_page(MOTO_UI_PAGE_NAVIGATION);
  state.mode = MOTO_UI_OFFLINE;
  state.online = 0;
  state.has_destination = 0;
  state.distance_to_maneuver_m = 0;
  moto_nav_ui_set_phone_connection(MOTO_UI_PHONE_CONNECTING);
  moto_nav_ui_set_state(&state);
  dump("23_connecting", dir);

  moto_nav_ui_set_phone_connection(MOTO_UI_PHONE_OFFLINE);
  dump("24_offline", dir);

  moto_nav_ui_set_phone_connection(MOTO_UI_PHONE_ONLINE);

  // --- Shutdown frame ---------------------------------------------------
  moto_nav_ui_show_power_off_screen();
  pump(120);
  dump("25_power_off", dir);

  std::printf("dumped %d frames to %s\n", counters, dir.c_str());
  return 0;
}

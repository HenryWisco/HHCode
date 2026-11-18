#include "vex.h"

namespace hh {
namespace odom_map {

float field_size = 144;
float center_x = 0;
float center_y = 0;

// The trail is a ring: once it is full, the oldest point is overwritten.
static const int MAX_TRAIL = 300;
static float trail_x[MAX_TRAIL];
static float trail_y[MAX_TRAIL];
static int trail_start = 0;
static int trail_count = 0;

// The map is a square on the left of the screen.
static const int MAP_LEFT = 6, MAP_TOP = 34, MAP_SIZE = 200;

static bool has_odom() { return chassis.drive_setup != ZERO_TRACKER_NO_ODOM; }

void clear_trail() {
  trail_start = 0;
  trail_count = 0;
}

int trail_points() { return trail_count; }

void update() {
  static uint32_t last_check = 0;
  uint32_t time = now_ms();
  if (time - last_check < 20 || !has_odom()) return;
  last_check = time;

  float x = chassis.get_X_position();
  float y = chassis.get_Y_position();
  if (trail_count > 0) {
    int newest = (trail_start + trail_count - 1) % MAX_TRAIL;
    if (hypot(x - trail_x[newest], y - trail_y[newest]) < 1.5) return;
  }
  int slot = (trail_start + trail_count) % MAX_TRAIL;
  trail_x[slot] = x;
  trail_y[slot] = y;
  if (trail_count < MAX_TRAIL) {
    trail_count++;
  } else {
    trail_start = (trail_start + 1) % MAX_TRAIL;
  }
}

// Field inches to screen pixels. Y goes up on the field and down on the screen.
static int screen_x(float x) { return MAP_LEFT + (int)((x - center_x + field_size / 2) / field_size * MAP_SIZE); }
static int screen_y(float y) { return MAP_TOP + MAP_SIZE - (int)((y - center_y + field_size / 2) / field_size * MAP_SIZE); }

static bool on_map(float x, float y) {
  return fabs(x - center_x) <= field_size / 2 && fabs(y - center_y) <= field_size / 2;
}

static void line(float x1, float y1, float x2, float y2) {
  if (!on_map(x1, y1) || !on_map(x2, y2)) return;
  Brain.Screen.drawLine(screen_x(x1), screen_y(y1), screen_x(x2), screen_y(y2));
}

void draw() {
  Brain.Screen.clearScreen();
  Brain.Screen.setFont(vex::fontType::mono20);
  Brain.Screen.setFillColor(vex::color::black);
  Brain.Screen.setPenColor(vex::color::white);
  Brain.Screen.printAt(10, 22, true, "Odometry map  (tap to go back)");

  // The field, with a line every tile (a sixth of the field).
  Brain.Screen.setPenColor(vex::color(0x616161));
  Brain.Screen.setFillColor(vex::color(0x212121));
  Brain.Screen.drawRectangle(MAP_LEFT, MAP_TOP, MAP_SIZE, MAP_SIZE);
  Brain.Screen.setPenColor(vex::color(0x424242));
  for (int i = 1; i < 6; i++) {
    int offset = i * MAP_SIZE / 6;
    Brain.Screen.drawLine(MAP_LEFT + offset, MAP_TOP, MAP_LEFT + offset, MAP_TOP + MAP_SIZE);
    Brain.Screen.drawLine(MAP_LEFT, MAP_TOP + offset, MAP_LEFT + MAP_SIZE, MAP_TOP + offset);
  }
  Brain.Screen.setFillColor(vex::color::black);

  float x = chassis.get_X_position();
  float y = chassis.get_Y_position();
  float heading = chassis.get_absolute_heading();

  if (has_odom()) {
    // Trail, oldest to newest.
    Brain.Screen.setPenColor(vex::color(0x29B6F6));
    for (int i = 1; i < trail_count; i++) {
      int a = (trail_start + i - 1) % MAX_TRAIL;
      int b = (trail_start + i) % MAX_TRAIL;
      line(trail_x[a], trail_y[a], trail_x[b], trail_y[b]);
    }

    // The robot: an arrow pointing the way it faces. Heading 0 is up the
    // screen (+Y) and goes clockwise, the same as JAR-Template.
    float size = field_size / 12.0;
    float forward_x = sin(to_rad(heading)), forward_y = cos(to_rad(heading));
    float right_x = forward_y, right_y = -forward_x;
    float nose_x = x + forward_x * size, nose_y = y + forward_y * size;
    float left_x = x - forward_x * size * 0.6 - right_x * size * 0.6;
    float left_y = y - forward_y * size * 0.6 - right_y * size * 0.6;
    float back_right_x = x - forward_x * size * 0.6 + right_x * size * 0.6;
    float back_right_y = y - forward_y * size * 0.6 + right_y * size * 0.6;
    Brain.Screen.setPenColor(vex::color(0xFFEB3B));
    line(nose_x, nose_y, left_x, left_y);
    line(nose_x, nose_y, back_right_x, back_right_y);
    line(left_x, left_y, back_right_x, back_right_y);
  }

  // Numbers on the right.
  Brain.Screen.setPenColor(vex::color::white);
  Brain.Screen.setFont(vex::fontType::mono30);
  if (has_odom()) {
    Brain.Screen.printAt(222, 80, false, "X %7.1f in", x);
    Brain.Screen.printAt(222, 120, false, "Y %7.1f in", y);
  }
  Brain.Screen.printAt(222, 160, false, "H %7.1f deg", heading);
  Brain.Screen.setFont(vex::fontType::mono20);
  Brain.Screen.setPenColor(vex::color(0xBDBDBD));
  if (!has_odom()) {
    Brain.Screen.printAt(222, 70, false, "No odometry in this");
    Brain.Screen.printAt(222, 92, false, "chassis setup.");
  } else if (!on_map(x, y)) {
    Brain.Screen.printAt(222, 200, false, "Robot is off the map");
  } else if (chassis.is_mirrored()) {
    Brain.Screen.printAt(222, 200, false, "Mirrored auton");
  }
}

}  // namespace odom_map
}  // namespace hh

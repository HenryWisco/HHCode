#include "vex.h"

namespace hh {

namespace walls {

float max_correction = 8;
float max_angle = 20;
float max_range = 70;

// A plain array (not a vector) so sensors declared as globals can register
// themselves before main() starts.
static const int MAX_SENSORS = 8;
static WallSensor* sensors[MAX_SENSORS];
static int sensor_total = 0;

int sensor_count() { return sensor_total; }

WallSensor* sensor_at(int index) {
  if (index < 0 || index >= sensor_total) return nullptr;
  return sensors[index];
}

}  // namespace walls

WallSensor::WallSensor(vex::distance& sensor, Facing facing, float forward_offset, float right_offset)
    : sensor_(&sensor), facing_(facing), forward_offset_(forward_offset), right_offset_(right_offset) {
  if (walls::sensor_total < walls::MAX_SENSORS) walls::sensors[walls::sensor_total++] = this;
}

float WallSensor::inches() {
  if (!sensor_->installed() || !sensor_->isObjectDetected()) return -1;
  float reading = sensor_->objectDistance(vex::distanceUnits::in);
  if (reading <= 0 || reading > walls::max_range) return -1;
  return reading;
}

bool WallSensor::measure(bool& is_x, float& coordinate, float& reading) {
  if (chassis.drive_setup == ZERO_TRACKER_NO_ODOM) {
    status = "no odometry";
    return false;
  }
  // Where the sensor is and which way it points, relative to the robot.
  // Angles are clockwise from straight ahead, the same as headings.
  float facing_angle = facing_ * 90.0 + angle_offset;
  float right_offset = right_offset_;
  // A mirrored auton runs as the robot's mirror image: its left sensor is
  // the mirror robot's right sensor.
  if (chassis.is_mirrored()) {
    facing_angle = -facing_angle;
    right_offset = -right_offset;
  }

  // Which way the sensor points on the field (0 = toward +Y, 90 = toward +X).
  float heading = chassis.get_absolute_heading();
  float beam_angle = heading + facing_angle;

  // The beam must hit the wall it is aimed at close to square. At a steep
  // angle the reading is unreliable, and near a corner it may be hitting
  // the neighbouring wall instead.
  int quarter_turns = (int)floor(beam_angle / 90.0 + 0.5);
  float off_square = beam_angle - quarter_turns * 90.0;
  if (fabs(off_square) > walls::max_angle) {
    status = "not square to a wall";
    return false;
  }
  int wall = ((quarter_turns % 4) + 4) % 4;   // 0 = +Y wall, 1 = +X, 2 = -Y, 3 = -X

  reading = inches();
  if (reading < 0) {
    status = "sees nothing";
    return false;
  }

  // The beam's direction and the sensor's position on the field, both
  // measured from the robot's center.
  float beam_x = sin(to_rad(beam_angle)), beam_y = cos(to_rad(beam_angle));
  float forward_x = sin(to_rad(heading)), forward_y = cos(to_rad(heading));
  float sensor_x = forward_offset_ * forward_x + right_offset * forward_y;
  float sensor_y = forward_offset_ * forward_y - right_offset * forward_x;

  // center + sensor position + reading x beam direction = the wall.
  // Solve that for the center.
  float half_field = odom_map::field_size / 2.0;
  float current;
  if (wall == 0 || wall == 2) {
    is_x = false;
    float wall_y = odom_map::center_y + (wall == 0 ? half_field : -half_field);
    coordinate = wall_y - reading * beam_y - sensor_y;
    current = chassis.get_Y_position();
  } else {
    is_x = true;
    float wall_x = odom_map::center_x + (wall == 1 ? half_field : -half_field);
    coordinate = wall_x - reading * beam_x - sensor_x;
    current = chassis.get_X_position();
  }

  // A big jump almost always means something was between the sensor and the
  // wall. Believing it would be worse than not correcting at all.
  if (walls::max_correction > 0 && fabs(coordinate - current) > walls::max_correction) {
    status = "too far from expected";
    return false;
  }
  status = "ok";
  return true;
}

// Moves the tracked position to (x, y) without touching the heading.
static void set_position(float x, float y) {
  chassis.odom.set_position(x, y, chassis.get_absolute_heading(),
                            chassis.get_ForwardTracker_position(), chassis.get_SidewaysTracker_position());
}

bool WallSensor::reset() {
  bool is_x;
  float coordinate, reading;
  if (!measure(is_x, coordinate, reading)) return false;
  if (is_x) {
    set_position(coordinate, chassis.get_Y_position());
  } else {
    set_position(chassis.get_X_position(), coordinate);
  }
  return true;
}

int wall_reset() {
  // The best (shortest) usable reading for each of X and Y.
  bool have_x = false, have_y = false;
  float best_x = 0, best_y = 0, reading_x = 0, reading_y = 0;

  for (int i = 0; i < walls::sensor_total; i++) {
    bool is_x;
    float coordinate, reading;
    if (!walls::sensors[i]->measure(is_x, coordinate, reading)) continue;
    if (is_x && (!have_x || reading < reading_x)) {
      have_x = true;
      best_x = coordinate;
      reading_x = reading;
    }
    if (!is_x && (!have_y || reading < reading_y)) {
      have_y = true;
      best_y = coordinate;
      reading_y = reading;
    }
  }

  if (!have_x && !have_y) return 0;
  set_position(have_x ? best_x : chassis.get_X_position(), have_y ? best_y : chassis.get_Y_position());
  return (have_x ? 1 : 0) + (have_y ? 1 : 0);
}

}  // namespace hh

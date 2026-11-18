// hh::Chassis: the one-line chassis setup.
//
// A Chassis is a short-lived "order form". tank(), mecanum() or x_drive()
// start the form, the optional add-ons (.odom(), .forward_tracker(), ...)
// fill in more of it, and when the line ends the form is turned into
// JAR-Template's real Drive object by operator Drive() at the bottom of
// this file.

#include "vex.h"

namespace hh {

// Which drive type the builder made: -1 = the builder wasn't used.
static int built_kind = -1;

// Turns a port number as a student writes it (1 to 21, negative = reversed)
// into the number the VEX API uses (PORT1 is 0, PORT2 is 1, ...). A number
// outside 1 to 21 is a typing mistake; it is pulled back into range so it
// can't crash the program, and the ports page will show nothing plugged in.
static int port_index(int port) {
  int number = abs(port);
  if (number < 1) number = 1;
  if (number > 21) number = 21;
  return number - 1;
}

// Top speed of each motor cartridge, used to work out the gear ratio.
static float cartridge_rpm(vex::gearSetting cartridge) {
  if (cartridge == vex::gearSetting::ratio36_1) return 100;
  if (cartridge == vex::gearSetting::ratio6_1) return 600;
  return 200;
}

// Starts the form for a tank drive: just write everything down.
Chassis Chassis::tank(std::initializer_list<int> left_ports, std::initializer_list<int> right_ports,
                      int imu_port, float wheel_diameter, vex::gearSetting cartridge, float wheel_rpm) {
  Chassis c;
  c.kind_ = 0;
  c.left_.assign(left_ports.begin(), left_ports.end());
  c.right_.assign(right_ports.begin(), right_ports.end());
  c.imu_port_ = imu_port;
  c.wheel_diameter_ = wheel_diameter;
  c.cartridge_ = cartridge;
  c.wheel_rpm_ = wheel_rpm;
  return c;
}

// Mecanum and X-drive share this. JAR-Template wants a left and a right
// motor group as well as the four separate wheels, so the left group is
// the two left wheels and the right group the two right wheels.
Chassis Chassis::holonomic(int kind, int lf, int rf, int lb, int rb, int imu_port, float wheel_diameter,
                           vex::gearSetting cartridge, float wheel_rpm) {
  Chassis c;
  c.kind_ = kind;
  c.left_ = {lf, lb};
  c.right_ = {rf, rb};
  c.imu_port_ = imu_port;
  c.wheel_diameter_ = wheel_diameter;
  c.cartridge_ = cartridge;
  c.wheel_rpm_ = wheel_rpm;
  return c;
}

Chassis Chassis::mecanum(int left_front, int right_front, int left_back, int right_back,
                         int imu_port, float wheel_diameter, vex::gearSetting cartridge, float wheel_rpm) {
  return holonomic(1, left_front, right_front, left_back, right_back, imu_port, wheel_diameter, cartridge, wheel_rpm);
}

Chassis Chassis::x_drive(int left_front, int right_front, int left_back, int right_back,
                         int imu_port, float wheel_diameter, vex::gearSetting cartridge, float wheel_rpm) {
  return holonomic(2, left_front, right_front, left_back, right_back, imu_port, wheel_diameter, cartridge, wheel_rpm);
}

// The add-ons. Each one notes its numbers and returns the same form
// (*this) so another add-on can be chained after it.
Chassis& Chassis::odom(float track_width) {
  track_width_ = track_width;
  return *this;
}

// JAR-Template reverses a tracking wheel by making its diameter
// negative, so a negative port number is turned into that here.
Chassis& Chassis::forward_tracker(int port, float diameter, float offset) {
  forward_ = true;
  forward_is_encoder_ = false;
  forward_port_ = port_index(port);
  forward_diameter_ = port < 0 ? -diameter : diameter;
  forward_offset_ = offset;
  return *this;
}

Chassis& Chassis::sideways_tracker(int port, float diameter, float offset) {
  sideways_ = true;
  sideways_is_encoder_ = false;
  sideways_port_ = port_index(port);
  sideways_diameter_ = port < 0 ? -diameter : diameter;
  sideways_offset_ = offset;
  return *this;
}

// 'A' or 'a' -> 1 ... 'H' -> 8, the numbering JAR-Template uses for 3-wire ports.
static int letter_number(char letter) {
  if (letter >= 'a' && letter <= 'h') return letter - 'a' + 1;
  if (letter >= 'A' && letter <= 'H') return letter - 'A' + 1;
  return 1;
}

Chassis& Chassis::forward_encoder(char port_letter, float diameter, float offset) {
  forward_ = true;
  forward_is_encoder_ = true;
  forward_port_ = letter_number(port_letter);
  forward_diameter_ = diameter;
  forward_offset_ = offset;
  return *this;
}

Chassis& Chassis::sideways_encoder(char port_letter, float diameter, float offset) {
  sideways_ = true;
  sideways_is_encoder_ = true;
  sideways_port_ = letter_number(port_letter);
  sideways_diameter_ = diameter;
  sideways_offset_ = offset;
  return *this;
}

Chassis& Chassis::gyro_scale(float degrees_per_turn) {
  gyro_scale_ = degrees_per_turn;
  return *this;
}

// Builds the real chassis. This runs once, when the line
// "Drive chassis = hh::Chassis::tank(...);" finishes.
Chassis::operator Drive() {
  built_kind = kind_;

  // The motors are made here and never deleted: the chassis uses them for as
  // long as the program runs.
  vex::motor_group left_group, right_group;
  for (int port : left_) {
    left_group(*new vex::motor(port_index(port), cartridge_, port < 0));
  }
  for (int port : right_) {
    right_group(*new vex::motor(port_index(port), cartridge_, port < 0));
  }

  // Which of JAR-Template's setups this is.
  bool encoders = forward_ ? forward_is_encoder_ : sideways_is_encoder_;
  enum ::drive_setup setup = ZERO_TRACKER_NO_ODOM;
  float forward_offset = forward_offset_;
  if (kind_ != 0) {
    if (forward_ && sideways_) {
      setup = encoders ? HOLONOMIC_TWO_ENCODER : HOLONOMIC_TWO_ROTATION;
    } else {
      setup = HOLONOMIC_ZERO_TRACKER_ODOM;
    }
  } else if (forward_ && sideways_) {
    setup = encoders ? TANK_TWO_ENCODER : TANK_TWO_ROTATION;
  } else if (forward_) {
    setup = encoders ? TANK_ONE_FORWARD_ENCODER : TANK_ONE_FORWARD_ROTATION;
  } else if (sideways_) {
    // The right side of the drive stands in for the forward tracker.
    setup = encoders ? TANK_ONE_SIDEWAYS_ENCODER : TANK_ONE_SIDEWAYS_ROTATION;
    forward_offset = track_width_ / 2.0;
  } else if (track_width_ > 0) {
    setup = ZERO_TRACKER_ODOM;
    forward_offset = track_width_ / 2.0;
  }

  // JAR-Template always makes four "holonomic" motors. On a tank drive they
  // aren't used, so point them at the first and last motor of each side with
  // the same settings those motors already have.
  int front_left = left_.empty() ? 1 : left_.front();
  int front_right = right_.empty() ? 1 : right_.front();
  int back_left = left_.empty() ? 1 : left_.back();
  int back_right = right_.empty() ? 1 : right_.back();
  int ports[4] = {port_index(front_left), port_index(front_right), port_index(back_left), port_index(back_right)};
  bool reversed[4] = {front_left < 0, front_right < 0, back_left < 0, back_right < 0};

  float ratio = wheel_rpm_ > 0 ? wheel_rpm_ / cartridge_rpm(cartridge_) : 1;

  return Drive(setup, left_group, right_group, port_index(imu_port_), wheel_diameter_, ratio, gyro_scale_,
               ports, reversed, cartridge_,
               forward_port_, forward_diameter_, forward_offset,
               sideways_port_, sideways_diameter_, sideways_offset_);
}

// mecanum() and x_drive() already know the drive type, so the team
// doesn't have to set hh::driver::drive_type as well. hh::init() calls
// this after configure_robot().
void apply_chassis_drive_type() {
  if (built_kind == 1) driver::drive_type = MECANUM;
  if (built_kind == 2) driver::drive_type = X_DRIVE;
}

}  // namespace hh

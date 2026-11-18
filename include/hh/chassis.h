#pragma once
#include <initializer_list>
#include <vector>
#include "v5.h"
#include "v5_vcs.h"

class Drive;

namespace hh {

/**
 * The short way to set up the chassis: one line instead of the long
 * `Drive chassis(...)` block, and no drive motors to declare in section 1.
 *
 *   Drive chassis = hh::Chassis::tank({1, -2, 3}, {-4, 5, -6}, 10, 3.25, ratio6_1, 450);
 *
 * That reads: left motors on ports 1, 2 and 3 (2 is reversed), right motors
 * on 4, 5 and 6 (4 and 6 reversed), inertial sensor on port 10, 3.25" wheels,
 * blue motors, and the wheels turn at 450 rpm.
 *
 * - Ports are plain numbers, 1 to 21. A minus sign means reversed. Every
 *   drive motor must push the robot forward when it spins forward.
 * - Cartridge: ratio36_1 = red (100 rpm), ratio18_1 = green (200 rpm),
 *   ratio6_1 = blue (600 rpm).
 * - Wheel rpm = cartridge rpm x motor gear teeth / wheel gear teeth. A blue
 *   motor with a 36-tooth gear driving a 48-tooth gear on the wheel is
 *   600 x 36 / 48 = 450. Direct drive is just the cartridge rpm.
 *
 * Mecanum and X-drive list the four wheels by where they are:
 *
 *   Drive chassis = hh::Chassis::mecanum(1, -2, 3, -4, 10, 4, ratio18_1, 200);
 *   Drive chassis = hh::Chassis::x_drive(1, -2, 3, -4, 10, 3.25, ratio6_1, 300);
 *
 * Add odometry by chaining more onto the end (see each function below):
 *
 *   Drive chassis = hh::Chassis::tank({1, 2}, {-3, -4}, 10, 3.25, ratio6_1, 360)
 *                     .forward_tracker(12, 2.75, -2)
 *                     .sideways_tracker(13, 2.75, 5.5);
 *
 * What you get is JAR-Template's normal `Drive`, so everything else in the
 * docs works the same. The mecanum() and x_drive() forms also set
 * hh::driver::drive_type for you.
 */
class Chassis {
public:
  /** Tank drive. Any number of motors per side. */
  static Chassis tank(std::initializer_list<int> left_ports, std::initializer_list<int> right_ports,
                      int imu_port, float wheel_diameter, vex::gearSetting cartridge, float wheel_rpm);
  /** Mecanum drive. Ports in the order left front, right front, left back, right back. */
  static Chassis mecanum(int left_front, int right_front, int left_back, int right_back,
                         int imu_port, float wheel_diameter, vex::gearSetting cartridge, float wheel_rpm);
  /** X-drive. Ports in the order left front, right front, left back, right back. */
  static Chassis x_drive(int left_front, int right_front, int left_back, int right_back,
                         int imu_port, float wheel_diameter, vex::gearSetting cartridge, float wheel_rpm);

  /**
   * Tank drive odometry without tracking wheels (uses the drive motors and
   * the inertial sensor). `track_width` is the distance in inches between
   * the left and right wheels. Also needed with only a sideways tracker.
   * Mecanum and X-drive get motor odometry by themselves and don't need this.
   */
  Chassis& odom(float track_width);

  /**
   * A tracking wheel on a rotation sensor that rolls when the robot drives
   * forward. `port` is 1 to 21, negative if it counts backwards. `offset` is
   * how far the wheel is to the right of the robot's center, in inches
   * (negative = left).
   */
  Chassis& forward_tracker(int port, float diameter, float offset);
  /**
   * A tracking wheel on a rotation sensor that rolls when the robot slides
   * sideways. `offset` is how far it is behind the robot's center, in inches
   * (negative = in front).
   */
  Chassis& sideways_tracker(int port, float diameter, float offset);
  /**
   * The same two tracking wheels on 3-wire encoders. `port_letter` is the
   * first of the encoder's two ports, 'A' to 'G'. Use a negative diameter if
   * it counts backwards. If the robot has two trackers, both must be the same
   * kind of sensor.
   */
  Chassis& forward_encoder(char port_letter, float diameter, float offset);
  Chassis& sideways_encoder(char port_letter, float diameter, float offset);

  /** What the inertial sensor reads after one full turn. Normally 360. */
  Chassis& gyro_scale(float degrees_per_turn);

  /** Builds the chassis. Runs by itself when you write `Drive chassis = ...`. */
  operator Drive();

private:
  Chassis() {}
  static Chassis holonomic(int kind, int lf, int rf, int lb, int rb, int imu_port, float wheel_diameter,
                           vex::gearSetting cartridge, float wheel_rpm);

  int kind_ = 0;   // 0 tank, 1 mecanum, 2 X-drive
  std::vector<int> left_;
  std::vector<int> right_;
  int imu_port_ = 1;
  float wheel_diameter_ = 3.25;
  vex::gearSetting cartridge_ = vex::gearSetting::ratio18_1;
  float wheel_rpm_ = 200;
  float gyro_scale_ = 360;
  float track_width_ = 0;
  bool forward_ = false, sideways_ = false;
  bool forward_is_encoder_ = false, sideways_is_encoder_ = false;
  int forward_port_ = 0, sideways_port_ = 0;
  float forward_diameter_ = 2.75, sideways_diameter_ = 2.75;
  float forward_offset_ = 0, sideways_offset_ = 0;
};

/** Used by hh::init(): applies the drive type a mecanum/X-drive Chassis asked for. */
void apply_chassis_drive_type();

}  // namespace hh

#pragma once
#include "vex.h"
#include <functional>
#include <vector>

// HHCode: this file is JAR-Template v1.2.0 with additions. Every change is
// marked with a "HHCode:" comment so it can be found when updating JAR.

// HHCode: added HOLONOMIC_ZERO_TRACKER_ODOM (mecanum / X-drive odom from the
// drive motor encoders, no tracking wheels).
enum drive_setup {ZERO_TRACKER_NO_ODOM, ZERO_TRACKER_ODOM, TANK_ONE_FORWARD_ENCODER, TANK_ONE_FORWARD_ROTATION,
TANK_ONE_SIDEWAYS_ENCODER, TANK_ONE_SIDEWAYS_ROTATION, TANK_TWO_ENCODER, TANK_TWO_ROTATION,
HOLONOMIC_TWO_ENCODER, HOLONOMIC_TWO_ROTATION, HOLONOMIC_ZERO_TRACKER_ODOM};

// HHCode: a point on a path for follow_path(), in inches.
struct path_point { float x; float y; };

// HHCode: what wait_until() measures for the motion that is running.
enum motion_kind {MOTION_DRIVE, MOTION_TURN, MOTION_ODOM, MOTION_STRAFE};

/**
 * Drive class supporting tank and holo drive, with or without odom.
 * Eight flavors of odom and six custom motion algorithms.
 */

class Drive
{
private:
  float wheel_diameter;
  float wheel_ratio;
  float gyro_scale;
  float drive_in_to_deg_ratio;
  float ForwardTracker_center_distance;
  float ForwardTracker_diameter;
  float ForwardTracker_in_to_deg_ratio;
  float SidewaysTracker_center_distance;
  float SidewaysTracker_diameter;
  float SidewaysTracker_in_to_deg_ratio;
  vex:: triport ThreeWire = vex::triport(vex::PORT22);

  // HHCode: state for async motions, wait_until(), slew and stall detection.
  int holonomic_ports[4];
  bool holonomic_reversed[4];
  bool next_async = false;
  bool next_chain = false;
  bool in_motion_task = false;
  bool motion_active = false;
  std::function<void()> pending_motion;
  vex::task motion_task;
  motion_kind progress_kind = MOTION_DRIVE;
  float progress_start_position = 0;
  float progress_start_rotation = 0;
  float progress_start_strafe = 0;
  float progress_start_X = 0;
  float progress_start_Y = 0;
  float stall_ms = 0;
  void record_progress_start(motion_kind kind);
  bool begin_motion(motion_kind kind, std::function<void()> motion);
  bool take_chain();
  void stop_motion_task();
  float slew(float target, float previous, float volts_per_second);
  bool check_stall(float commanded_voltage);
  float current_drive_voltage();
  float current_turn_voltage();
  static int motion_task_function();
  int next_turn_direction = 0;
  int take_turn_direction();
  bool use_backward_constants = false;
  float backward_max_voltage = 0;
  float backward_kp = 0;
  float backward_ki = 0;
  float backward_kd = 0;
  float backward_starti = 0;
  bool mirrored = false;
  void spin_holonomic(float lf, float rf, float lb, float rb);

public:
  drive_setup drive_setup = ZERO_TRACKER_NO_ODOM;
  motor_group DriveL;
  motor_group DriveR;
  inertial Gyro;
  motor DriveLF;
  motor DriveRF;
  motor DriveLB;
  motor DriveRB;
  rotation R_ForwardTracker;
  rotation R_SidewaysTracker;
  encoder E_ForwardTracker;
  encoder E_SidewaysTracker;

  float turn_max_voltage;
  float turn_kp;
  float turn_ki;
  float turn_kd;
  float turn_starti;

  float turn_settle_error;
  float turn_settle_time;
  float turn_timeout;

  float drive_min_voltage;
  float drive_max_voltage;
  float drive_kp;
  float drive_ki;
  float drive_kd;
  float drive_starti;

  float drive_settle_error;
  float drive_settle_time;
  float drive_timeout;

  float heading_max_voltage;
  float heading_kp;
  float heading_ki;
  float heading_kd;
  float heading_starti;

  float swing_max_voltage;
  float swing_kp;
  float swing_ki;
  float swing_kd;
  float swing_starti;

  float swing_settle_error;
  float swing_settle_time;
  float swing_timeout;

  float boomerang_lead;
  float boomerang_setback;

  Drive(enum::drive_setup drive_setup, motor_group DriveL, motor_group DriveR, int gyro_port, float wheel_diameter, float wheel_ratio, float gyro_scale, int DriveLF_port, int DriveRF_port, int DriveLB_port, int DriveRB_port, int ForwardTracker_port, float ForwardTracker_diameter, float ForwardTracker_center_distance, int SidewaysTracker_port, float SidewaysTracker_diameter, float SidewaysTracker_center_distance);

  void drive_with_voltage(float leftVoltage, float rightVoltage);

  float get_absolute_heading();

  float get_left_position_in();

  float get_right_position_in();

  void set_turn_constants(float turn_max_voltage, float turn_kp, float turn_ki, float turn_kd, float turn_starti); 
  void set_drive_constants(float drive_max_voltage, float drive_kp, float drive_ki, float drive_kd, float drive_starti);
  void set_heading_constants(float heading_max_voltage, float heading_kp, float heading_ki, float heading_kd, float heading_starti);
  void set_swing_constants(float swing_max_voltage, float swing_kp, float swing_ki, float swing_kd, float swing_starti);

  void set_turn_exit_conditions(float turn_settle_error, float turn_settle_time, float turn_timeout);
  void set_drive_exit_conditions(float drive_settle_error, float drive_settle_time, float drive_timeout);
  void set_swing_exit_conditions(float swing_settle_error, float swing_settle_time, float swing_timeout);

  void turn_to_angle(float angle);
  void turn_to_angle(float angle, float turn_max_voltage);
  void turn_to_angle(float angle, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout);
  void turn_to_angle(float angle, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout, float turn_kp, float turn_ki, float turn_kd, float turn_starti);

  void drive_distance(float distance);
  void drive_distance(float distance, float heading);
  void drive_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage);
  void drive_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout);
  void drive_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti);

  void left_swing_to_angle(float angle);
  void left_swing_to_angle(float angle, float swing_max_voltage, float swing_settle_error, float swing_settle_time, float swing_timeout, float swing_kp, float swing_ki, float swing_kd, float swing_starti);
  void right_swing_to_angle(float angle);
  void right_swing_to_angle(float angle, float swing_max_voltage, float swing_settle_error, float swing_settle_time, float swing_timeout, float swing_kp, float swing_ki, float swing_kd, float swing_starti);
  
  Odom odom;
  float get_ForwardTracker_position();
  float get_SidewaysTracker_position();
  void set_coordinates(float X_position, float Y_position, float orientation_deg);
  void set_heading(float orientation_deg);
  void position_track();
  static int position_track_task();
  vex::task odom_task;
  float get_X_position();
  float get_Y_position();

  void drive_stop(vex::brakeType mode);

  void drive_to_point(float X_position, float Y_position);
  void drive_to_point(float X_position, float Y_position, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage);
  void drive_to_point(float X_position, float Y_position, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout);
  void drive_to_point(float X_position, float Y_position, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti);
  
  void drive_to_pose(float X_position, float Y_position, float angle);
  void drive_to_pose(float X_position, float Y_position, float angle, float lead, float setback, float drive_min_voltage);
  void drive_to_pose(float X_position, float Y_position, float angle, float lead, float setback, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage);
  void drive_to_pose(float X_position, float Y_position, float angle, float lead, float setback, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout);
  void drive_to_pose(float X_position, float Y_position, float angle, float lead, float setback, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti);
  
  void turn_to_point(float X_position, float Y_position);
  void turn_to_point(float X_position, float Y_position, float extra_angle_deg);
  void turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout);
  void turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout, float turn_kp, float turn_ki, float turn_kd, float turn_starti);
  
  void holonomic_drive_to_pose(float X_position, float Y_position);
  void holonomic_drive_to_pose(float X_position, float Y_position, float angle);
  void holonomic_drive_to_pose(float X_position, float Y_position, float angle, float drive_max_voltage, float heading_max_voltage);
  void holonomic_drive_to_pose(float X_position, float Y_position, float angle, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout);
  void holonomic_drive_to_pose(float X_position, float Y_position, float angle, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti);

  // HHCode: the constructor hh::Chassis uses (holonomic motors as port + reversed + cartridge).
  Drive(enum::drive_setup drive_setup, motor_group DriveL, motor_group DriveR, int gyro_port, float wheel_diameter, float wheel_ratio, float gyro_scale, const int holonomic_port[4], const bool holonomic_reverse[4], vex::gearSetting holonomic_cartridge, int ForwardTracker_port, float ForwardTracker_diameter, float ForwardTracker_center_distance, int SidewaysTracker_port, float SidewaysTracker_diameter, float SidewaysTracker_center_distance);

  void control_arcade();
  void control_tank();
  void control_holonomic();

  /*-------------------------------------------------------------------------*/
  /*  HHCode additions. See docs/GUIDE.md.                                   */
  /*-------------------------------------------------------------------------*/

  // Slew: the most the voltage may rise per second at the start of a motion.
  // Stops wheelies and wheel slip. 0 is off. Try 30 to 60.
  float drive_slew = 0;
  float turn_slew = 0;

  // chain() motions end as soon as the robot is within these distances
  // (inches / degrees) instead of stopping and settling.
  float drive_chain_error = 3;
  float turn_chain_error = 5;

  // Stall detection: a motion gives up when the drive is powered but hasn't
  // moved for this long (ms), e.g. pinned against a wall or another robot.
  // 0 is off. Try 300. `interfered` says whether the last motion ended this way.
  float stall_time = 0;
  bool interfered = false;

  // Mecanum wheels slip sideways. If strafe_distance(24) really goes 20
  // inches, set this to 20.0 / 24.0.
  float strafe_multiplier = 1;

  // The next motion returns right away and runs in the background:
  //   chassis.async().drive_distance(36);
  Drive& async();
  // The next motion flows into the one after it without stopping:
  //   chassis.chain().drive_distance(24);
  Drive& chain();
  // Waits for a background motion to finish.
  void wait();
  // Waits until a background motion has covered this many inches (drives and
  // strafes) or degrees (turns and swings), or has finished.
  void wait_until(float amount);
  bool is_moving();
  float motion_progress();
  // Stops a background motion and brakes the drive.
  void cancel_motion();

  // Mecanum / X-drive: strafe sideways using the drive motor encoders.
  // Positive is to the right.
  float get_strafe_position_in();
  void strafe_distance(float distance);
  void strafe_distance(float distance, float heading);
  void strafe_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage);
  void strafe_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti);

  // Odom robots: drive smoothly through a list of points (pure pursuit).
  // The robot aims at a spot `lookahead` inches ahead of itself on the path;
  // bigger is smoother, smaller follows the path more tightly.
  //   chassis.follow_path({{0, 24}, {24, 48}, {48, 48}});
  float path_lookahead = 12;
  void follow_path(std::vector<path_point> path);
  void follow_path(std::vector<path_point> path, float lookahead);
  void follow_path(std::vector<path_point> path, float lookahead, bool backwards);
  void follow_path(std::vector<path_point> path, float lookahead, bool backwards, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage);
  void follow_path(std::vector<path_point> path, float lookahead, bool backwards, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti);

  // The next turn_to_angle() goes this way round even if the other is shorter:
  //   chassis.clockwise().turn_to_angle(270);
  Drive& clockwise();
  Drive& counterclockwise();

  // Separate constants for driving backward (max voltage, kP, kI, kD, startI).
  // Used by drive_distance() with a negative distance.
  void set_drive_backward_constants(float max_voltage, float kp, float ki, float kd, float starti);
  void clear_drive_backward_constants();

  // Rounds the corners of follow_path() paths. 0 is off; try 0.8.
  float path_smoothing = 0;
  std::vector<path_point> smooth_path(std::vector<path_point> path, float smoothing);

  // Mirrored autons: the chassis acts as its own mirror image (left and right
  // swapped, headings counted the other way), so a routine written for one
  // side of the field runs on the other. hh::selector::add_mirrored() turns
  // this on and off for you.
  void set_mirrored(bool mirror);
  bool is_mirrored();
  // The inertial sensor's rotation in degrees, not wrapped to 0-360.
  float get_rotation();

  // PTO: motors that are driving with each side right now. hh::Pto fills
  // these in; spin_side() and stop_side() run a whole side including them.
  std::vector<vex::motor*> pto_left;
  std::vector<vex::motor*> pto_right;
  void spin_side(bool left_side, float voltage);
  void stop_side(bool left_side, vex::brakeType mode);

  // Mecanum / X-drive: tell the chassis which cartridge the four wheel motors
  // use, so strafe distances are measured correctly. Not needed for green
  // (ratio18_1) motors.
  void set_holonomic_cartridge(vex::gearSetting cartridge);
};
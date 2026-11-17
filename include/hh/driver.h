#pragma once

namespace hh {

/** What kind of drivetrain the robot has. */
enum DriveType {
  TANK,      // wheels on the left and right sides (most robots)
  MECANUM,   // four mecanum wheels
  X_DRIVE    // four omni wheels at 45 degrees
};

/** How the joysticks drive the robot. */
enum DriveStyle {
  TANK_STICKS,      // left stick = left side, right stick = right side
  SPLIT_ARCADE,     // left stick = forward/back, right stick = turn
  LEFT_ARCADE,      // left stick does both
  RIGHT_ARCADE,     // right stick does both
  FLIPPED_ARCADE,   // right stick = forward/back, left stick = turn
  HOLONOMIC,        // left stick = move any direction, right stick = turn
  FIELD_CENTRIC     // like HOLONOMIC, but "up" is always away from the driver
};

/**
 * Driver control settings. Set these in configure_robot() in robot-config.cpp.
 * HOLONOMIC and FIELD_CENTRIC need a MECANUM or X_DRIVE drive_type.
 */
namespace driver {

extern DriveType drive_type;
extern DriveStyle style;

/** Stick movement smaller than this (percent) is ignored. */
extern float stick_deadband;
/**
 * Joystick curves. 0 is a straight line. Bigger numbers (try 2 to 5) give
 * finer control at low speed while keeping full speed at full stick.
 */
extern float throttle_curve;
extern float turn_curve;
/** Scales turning. 0.8 makes the robot turn at most 80% speed. */
extern float turn_scale;
/** Scales strafing on mecanum and X-drives. */
extern float strafe_scale;
/**
 * Active brake: when the sticks are released the drive resists being pushed.
 * 0 is off. Try 0.1; raise it for a firmer hold, lower it if the robot shakes.
 */
extern float active_brake_kp;
/** Motor temperature (C) that sets off a controller rumble. 0 turns it off. */
extern float overheat_temp;
/**
 * Lets the driver adjust the curves while driving: Left/Right arrows change
 * the throttle curve, Y/A change the turn curve. Values show on the controller
 * and are saved to the SD card. Turn this off for competition.
 */
extern bool curve_tuning;

/** Call every loop in usercontrol(). Runs the button bindings and the drive. */
void update();
/** Drives from the joysticks only. update() calls this for you. */
void drive();
/** Makes the direction the robot faces right now "away from the driver". */
void reset_field_centric();
/** The joystick curve. `input` is -100 to 100. */
float curve(float input, float strength);

void check_overheat();
void load_saved_curves();

}  // namespace driver
}  // namespace hh

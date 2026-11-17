#pragma once
#include "vex.h"

namespace hh {

/**
 * Positioner: anything that moves to a position and stays there.
 * Lifts, arms, wrists.
 *
 * Set it up in robot-config.cpp:
 *   hh::Positioner lift = hh::Positioner(motor_group(LiftMotor), 5);   // 5 = gear ratio
 *
 * Teach it some named positions, in degrees of the mechanism
 * (do this once, in pre_auton):
 *   lift.add_preset("down", 0);
 *   lift.add_preset("high", 110);
 *
 * Then use it anywhere:
 *   lift.go_to("high");     // starts moving, your code keeps running
 *   lift.wait();            // waits here until it arrives
 *   lift.go_to(45);         // you can also use a number of degrees
 *
 * Wherever the mechanism is when the program starts counts as 0 degrees,
 * so always start the robot with the lift all the way down.
 */
class Positioner
{
public:
  motor_group motors;

  /** Speed used when moving to a position, in percent. */
  float speed_percent = 100;

  /** How close, in degrees, counts as "there". */
  float settle_error = 3;

  /** wait() gives up after this long, in milliseconds, so a stuck lift can't freeze an auton. */
  float timeout = 3000;

  Positioner(motor_group motors);
  Positioner(motor_group motors, float gear_ratio);

  void add_preset(const char *name, float degrees);

  void go_to(const char *preset_name);
  void go_to(float degrees);

  /** Waits until the mechanism arrives, or until the timeout runs out. */
  void wait();

  /** True when the mechanism is within settle_error of its target. */
  bool is_done();

  /** Current position in degrees of the mechanism. */
  float get_position();

  /** Makes the current position count as 0 degrees. */
  void zero();

  /**
   * For driver control. Moves at a voltage out of 12 (negative goes down).
   * manual(0) holds the mechanism where it is.
   */
  void manual(float voltage);

  /** Stops and holds the current position. */
  void hold();

private:
  static const int max_presets = 10;
  const char *preset_names[max_presets];
  float preset_degrees[max_presets];
  int preset_count = 0;

  float gear_ratio = 1;
  float target = 0;
  bool moving_manually = false;
};

}

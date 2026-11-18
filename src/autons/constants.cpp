#include "vex.h"

/**
 * The tuning numbers for autonomous driving. main.cpp calls
 * default_constants() once when the program starts.
 *
 * These numbers decide how hard the robot pushes toward a target and when
 * it decides it has arrived. The ones here are JAR-Template's starting
 * values; every robot needs its own. docs/GUIDE.md (section 5, "Tuning")
 * walks through tuning them, and the live tuner (hh::tuner) lets you change
 * them from the controller without downloading.
 *
 * What the five numbers in each "constants" line mean:
 *
 *   max voltage  the most power the motion may use, out of 12
 *   kP           how hard to push for each inch (or degree) still to go.
 *                Too low: slow, stops short. Too high: overshoots, shakes.
 *   kI           a slow extra push that builds up when the robot sits just
 *                short of the target. Usually tiny, or 0.
 *   kD           a brake that works harder the faster the robot approaches.
 *                Raise it to stop overshoot.
 *   startI       kI only switches on once the robot is this close to the
 *                target (inches or degrees).
 *
 * There are four sets:
 *
 *   drive    driving a distance
 *   heading  keeping straight while driving
 *   turn     turning on the spot
 *   swing    turning with only one side of the drive
 */

void default_constants(){
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  chassis.set_heading_constants(6, .4, 0, 1, 0);
  chassis.set_turn_constants(12, .4, .03, 3, 15);
  chassis.set_swing_constants(12, .3, .001, 2, 15);

  // When is a motion finished? Each set is (settle_error, settle_time, timeout):
  //   settle_error  close enough: within this many inches (or degrees)...
  //   settle_time   ...for this many milliseconds in a row
  //   timeout       give up after this many milliseconds, whatever happens,
  //                 so one stuck motion can't use up the whole autonomous
  chassis.set_drive_exit_conditions(1.5, 300, 5000);
  chassis.set_turn_exit_conditions(1, 300, 3000);
  chassis.set_swing_exit_conditions(1, 300, 3000);

  // HHCode extras. See docs/GUIDE.md, "Other auton settings".
  // Slew: the most the voltage may rise per second at the start of a motion.
  // 0 is off. Try 30 to 60 if the robot wheelies or spins its wheels.
  chassis.drive_slew = 0;
  chassis.turn_slew = 0;
  // Stall detection: a motion gives up after the robot has been stuck for this
  // many ms. 0 is off. Try 300.
  chassis.stall_time = 0;
  // chain() motions end when this close (inches / degrees) to the target.
  chassis.drive_chain_error = 3;
  chassis.turn_chain_error = 5;
}

/**
 * Constants for motions that drive to a point on the field (drive_to_point,
 * drive_to_pose, follow_path). Call odom_constants() at the top of an auton
 * that uses them. Those motions steer while they drive, so they work better
 * a little slower and with a bigger "close enough" than a plain drive.
 */

void odom_constants(){
  default_constants();                 // start from the normal numbers...
  chassis.heading_max_voltage = 10;    // ...allow stronger steering
  chassis.drive_max_voltage = 8;       // ...drive a little slower
  chassis.drive_settle_error = 3;      // ...and accept ending within 3 inches
  chassis.boomerang_lead = .5;         // how curved drive_to_pose() paths are (0 to 1)
  chassis.drive_min_voltage = 0;       // lowest power while moving (raise to keep speed through a chain)
}

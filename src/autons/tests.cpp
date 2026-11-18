#include "vex.h"

/**
 * Test routines. Pick them on the Brain's auton screen to check that the
 * chassis is set up right and to tune the PID constants in
 * src/autons/constants.cpp. They are also good examples of how an auton is
 * written: one chassis command per line, run from top to bottom.
 *
 * Most are JAR-Template's. The last two are HHCode's.
 *
 * Distances are in inches. Angles are in degrees: 0 is the way the robot
 * faced when the program started, and they count up clockwise (90 is to the
 * right).
 */

/**
 * Drives forward three times (6 + 12 + 18 = 36 inches), then 36 back.
 * The robot should end exactly where it started.
 *
 * - Ends short or long every time: check the wheel diameter and gear ratio.
 * - Overshoots and comes back, or wobbles: tune the drive constants.
 */

void drive_test(){
  chassis.drive_distance(6);     // forward 6 inches
  chassis.drive_distance(12);    // forward 12 more
  chassis.drive_distance(18);    // forward 18 more
  chassis.drive_distance(-36);   // negative = backward, all the way home
}

/**
 * Turns to several angles and back to 0. The robot should end facing the
 * way it started.
 *
 * turn_to_angle() takes the heading to face, not how far to turn: after
 * turn_to_angle(90), turn_to_angle(225) turns another 135 degrees.
 */

void turn_test(){
  chassis.turn_to_angle(5);      // a tiny turn (hard to do accurately)
  chassis.turn_to_angle(30);
  chassis.turn_to_angle(90);     // facing right
  chassis.turn_to_angle(225);    // facing back and to the left
  chassis.turn_to_angle(0);      // facing forward again
}

/**
 * A swing turn powers one side of the drive and holds the other still, so
 * the robot pivots around one set of wheels. Should draw an S shape.
 */

void swing_test(){
  chassis.left_swing_to_angle(90);    // left side drives, robot pivots on the right wheels
  chassis.right_swing_to_angle(0);    // right side drives, back to facing forward
}

/**
 * A little of everything. Should end roughly where it started.
 */

void full_test(){
  chassis.drive_distance(24);
  chassis.turn_to_angle(-45);         // negative = counterclockwise (to the left)
  chassis.drive_distance(-36);
  chassis.right_swing_to_angle(-90);
  chassis.drive_distance(24);
  chassis.turn_to_angle(0);
}

/**
 * Doesn't drive. Prints the robot's tracked position on the Brain screen,
 * over and over, so you can push the robot around by hand and check that
 * the numbers change the way you expect:
 *
 * - Push it forward: Y should go up.
 * - Push it to the right: X should go up.
 * - Turn it clockwise: Heading should go up.
 *
 * (The odometry map does the same job with a picture: tap "Map>" on the
 * auton screen.)
 */

void odom_test(){
  chassis.set_coordinates(0, 0, 0);   // call this spot (0, 0), facing 0 degrees
  while(1){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5,20, "X: %f", chassis.get_X_position());
    Brain.Screen.printAt(5,40, "Y: %f", chassis.get_Y_position());
    Brain.Screen.printAt(5,60, "Heading: %f", chassis.get_absolute_heading());
    Brain.Screen.printAt(5,80, "ForwardTracker: %f", chassis.get_ForwardTracker_position());
    Brain.Screen.printAt(5,100, "SidewaysTracker: %f", chassis.get_SidewaysTracker_position());
    task::sleep(20);                  // 50 times a second is plenty
  }
}

/**
 * For tank drives with position tracking. Drives to the point (24, 24) and
 * back to (0, 0). Should end where it began. The first drive is a straight
 * line because the robot turns to face the point first; the drive home
 * curves because it doesn't.
 */

void tank_odom_test(){
  odom_constants();                   // gentler constants that suit drive_to_point()
  chassis.set_coordinates(0, 0, 0);
  chassis.turn_to_point(24, 24);      // face the point (X = 24, Y = 24)
  chassis.drive_to_point(24,24);      // drive to it
  chassis.drive_to_point(0,0);        // drive home, turning on the way
  chassis.turn_to_angle(0);
}

/**
 * For mecanum and X-drives with position tracking. Drives round a square
 * while turning a full circle. Should end where it started.
 *
 * holonomic_drive_to_pose(x, y, angle) goes to a point and ends up facing
 * `angle`, moving and turning at the same time.
 */

void holonomic_odom_test(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.holonomic_drive_to_pose(0, 18, 90);
  chassis.holonomic_drive_to_pose(18, 0, 180);
  chassis.holonomic_drive_to_pose(0, 18, 270);
  chassis.holonomic_drive_to_pose(0, 0, 0);
}

/**
 * HHCode: mecanum / X-drive only. Strafes right and comes back, without
 * needing odometry. If it goes too far or not far enough, adjust
 * chassis.strafe_multiplier.
 */

void strafe_test(){
  chassis.strafe_distance(24);    // 24 inches to the right
  chassis.strafe_distance(-24);   // and back to the left
}

/**
 * HHCode: shows async(), wait_until() and chain(). Drives 36 inches, printing
 * to the Brain when it passes 12 inches, then flows into a turn and comes back.
 *
 * This is the pattern for doing something part way through a drive, such as
 * starting an intake just before reaching a game piece.
 */

void async_test(){
  chassis.async().drive_distance(36);   // start driving and carry on to the next line at once
  chassis.wait_until(12);               // pause here until 12 inches have been covered
  Brain.Screen.printAt(5, 40, "Passed 12 inches");   // (an intake.spin() would go here)
  chassis.wait();                       // pause until the drive has finished
  chassis.turn_to_angle(180);
  chassis.chain().drive_distance(18);   // chain: don't stop and settle, flow into the next motion
  chassis.drive_distance(18);
  chassis.turn_to_angle(0);
}

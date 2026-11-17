#include "vex.h"

/**
 * The list of autons shown on the Brain screen, in order. The first one is
 * the default. Each line is (name, description, function).
 *
 * To add your own:
 *   1. Write the function in a new file in this folder, e.g. red_left.cpp.
 *   2. Declare it at the bottom of include/autons.h:   void red_left();
 *   3. Add a line here.
 *
 * The seven below are JAR-Template's test routines (src/autons/tests.cpp).
 * Use them to tune your robot, then delete the ones you don't need.
 */

void register_autons() {
  hh::selector::add("Drive Test", "Drives 6, 12, 18 inches forward, then 36 back. Should end where it started.", drive_test);
  hh::selector::add("Turn Test", "Turns to 5, 30, 90, 225, then back to 0 degrees.", turn_test);
  hh::selector::add("Swing Test", "Swings in an S shape using one side at a time.", swing_test);
  hh::selector::add("Full Test", "Drives, turns and swings. Should end roughly where it started.", full_test);
  hh::selector::add("Odom Test", "Doesn't move. Prints position to the screen; push the robot around to check it.", odom_test);
  hh::selector::add("Tank Odom Test", "Odom robots only. Drives to (24, 24) and back.", tank_odom_test);
  hh::selector::add("Holonomic Odom Test", "Mecanum / X-drive with odom only. Drives a square while turning.", holonomic_odom_test);
}

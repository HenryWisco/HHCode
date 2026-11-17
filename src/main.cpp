#include "vex.h"

using namespace vex;
competition Competition;

/*---------------------------------------------------------------------------*/
/*  You normally don't need to edit this file.                               */
/*                                                                           */
/*    Ports, chassis, mechanisms, buttons   ->  src/robot-config.cpp         */
/*    PID constants                         ->  src/autons/constants.cpp     */
/*    The list of autons                    ->  src/autons/auton_list.cpp    */
/*---------------------------------------------------------------------------*/

/**
 * Runs once when the program starts, before the match. Keep the robot still
 * until the Brain screen shows the auton selector: the inertial sensor is
 * calibrating.
 */

void pre_auton() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  default_constants();
  configure_robot();
  register_autons();
  hh::init();
}

/**
 * Runs the auton picked on the Brain screen.
 */

void autonomous(void) {
  hh::selector::run_selected();
}

/**
 * Driver control. hh::driver::update() reads the joysticks and runs every
 * button you set up in configure_robot(). Add anything extra inside the loop.
 */

void usercontrol(void) {
  while (1) {
    hh::driver::update();

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}

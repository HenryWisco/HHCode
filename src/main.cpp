#include "vex.h"

using namespace vex;
competition Competition;

/*---------------------------------------------------------------------------*/
/*                               HHCode main                                 */
/*                                                                           */
/*  Your robot is described in robot-config.cpp. Do that first.              */
/*                                                                           */
/*  This file has four parts, in the order you'll usually edit them:         */
/*    1. The auton list:    which autons show up on the Brain screen.        */
/*    2. setup_controls():  which controller buttons do what.                */
/*    3. setup_presets():   named positions for lifts and arms.              */
/*    4. usercontrol():     how the robot drives.                            */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*  1. The auton list                                                        */
/*                                                                           */
/*  Each line is {"name shown on the Brain", function_to_run}. Tap the       */
/*  Brain screen before a match to cycle through them. The first one in      */
/*  the list runs if nobody touches the screen.                              */
/*                                                                           */
/*  To add your own auton: write the function in a file in src/autons/,      */
/*  add it to include/autons.h, then add a line here.                        */
/*---------------------------------------------------------------------------*/

struct auton_entry
{
  const char *name;
  void (*run)();
};

auton_entry autons[] = {
  {"Example: drive + mechanisms", example_auton},
  {"Test: drive",                 drive_test},
  {"Test: turn",                  turn_test},
  {"Test: swing",                 swing_test},
  {"Test: full",                  full_test},
  {"Test: odom",                  odom_test},
  {"Test: tank odom",             tank_odom_test},
  {"Test: holonomic odom",        holonomic_odom_test},
};

const int auton_count = sizeof(autons)/sizeof(autons[0]);
int current_auton_selection = 0;
bool auto_started = false;

/*---------------------------------------------------------------------------*/
/*  2. Controller buttons                                                    */
/*                                                                           */
/*  One line per button. The three kinds are:                                */
/*    bind_hold    does something while the button is held, and something    */
/*                 else when you let go.                                     */
/*    bind_press   does something once per press.                            */
/*    bind_toggle  switches between two things on each press.                */
/*                                                                           */
/*  Button names: ButtonL1 ButtonL2 ButtonR1 ButtonR2 ButtonA ButtonB        */
/*  ButtonX ButtonY ButtonUp ButtonDown ButtonLeft ButtonRight               */
/*---------------------------------------------------------------------------*/

void setup_controls() {
  // Intake: hold R1 to pull in, hold R2 to push out.
  hh::bind_hold(Controller1.ButtonR1, [] { intake.spin(); },    [] { intake.stop(); });
  hh::bind_hold(Controller1.ButtonR2, [] { intake.reverse(); }, [] { intake.stop(); });

  // Lift: hold L1 to go up, hold L2 to go down. It holds its height when you let go.
  hh::bind_hold(Controller1.ButtonL1, [] { lift.manual(10); },  [] { lift.manual(0); });
  hh::bind_hold(Controller1.ButtonL2, [] { lift.manual(-8); },  [] { lift.manual(0); });

  // Lift presets: one press sends the lift to a saved height.
  hh::bind_press(Controller1.ButtonUp,   [] { lift.go_to("high"); });
  hh::bind_press(Controller1.ButtonDown, [] { lift.go_to("down"); });

  // Claw: press A to open or close.
  hh::bind_press(Controller1.ButtonA, [] { claw.toggle(); });
}

/*---------------------------------------------------------------------------*/
/*  3. Named positions                                                       */
/*                                                                           */
/*  Each line is ("name", degrees). Degrees are measured from wherever the   */
/*  mechanism is when the program starts, so always start with the lift      */
/*  all the way down. Find your numbers by trying them out.                  */
/*---------------------------------------------------------------------------*/

void setup_presets() {
  // CHANGE ME: these heights are examples.
  lift.add_preset("down", 0);
  lift.add_preset("mid",  60);
  lift.add_preset("high", 110);
}

/**
 * Runs once when the program starts, before autonomous. It shows the
 * auton selector on the Brain screen. Tapping the screen moves to the
 * next auton in the list.
 */

void pre_auton() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  default_constants();
  setup_presets();
  setup_controls();

  while(!auto_started){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5, 20, "HHCode v" HHCODE_VERSION);
    Brain.Screen.printAt(5, 40, "Battery Percentage:");
    Brain.Screen.printAt(5, 60, "%d", Brain.Battery.capacity());
    Brain.Screen.printAt(5, 80, "Chassis Heading Reading:");
    Brain.Screen.printAt(5, 100, "%f", chassis.get_absolute_heading());
    Brain.Screen.printAt(5, 120, "Selected Auton (tap to change):");
    Brain.Screen.printAt(5, 140, false, "%d: %s", current_auton_selection + 1, autons[current_auton_selection].name);
    if(Brain.Screen.pressing()){
      while(Brain.Screen.pressing()) {}
      current_auton_selection ++;
      if (current_auton_selection >= auton_count){
        current_auton_selection = 0;
      }
    }
    task::sleep(10);
  }
}

/**
 * Runs the auton that was selected on the Brain screen.
 */

void autonomous(void) {
  auto_started = true;
  autons[current_auton_selection].run();
}

/*---------------------------------------------------------------------------*/
/*  4. Driver control                                                        */
/*                                                                           */
/*  Pick ONE drive style and delete or comment out the others:               */
/*    chassis.control_arcade();     left stick forward/back, right stick     */
/*                                  turns.                                   */
/*    chassis.control_tank();       left stick is the left side, right       */
/*                                  stick is the right side.                 */
/*    chassis.control_holonomic();  X-drive and mecanum. Left stick drives   */
/*                                  and strafes, right stick turns.          */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  auto_started = true;
  while (1) {
    chassis.control_arcade();

    // Runs every button you set up in setup_controls().
    hh::run_bindings();

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

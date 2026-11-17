#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

/*---------------------------------------------------------------------------*/
/*  robot-config.cpp: everything about YOUR robot lives in this one file.     */
/*                                                                           */
/*    1. Motors and sensors                                                  */
/*    2. The chassis (drivetrain)                                            */
/*    3. Mechanisms (intake, lift, clamp, ...)                               */
/*    4. Driver control settings and buttons                                 */
/*                                                                           */
/*  Whatever you add in sections 1 and 3 also needs an "extern" line in      */
/*  include/robot-config.h.                                                  */
/*---------------------------------------------------------------------------*/

brain Brain;
controller Controller(primary);

/*---------------------------------------------------------------------------*/
/*  1. MOTORS AND SENSORS                                                    */
/*                                                                           */
/*  A motor is (port, cartridge, reversed):                                  */
/*    motor LeftFront = motor(PORT1, ratio6_1, false);                       */
/*  Cartridges: ratio36_1 = red 100 rpm, ratio18_1 = green 200 rpm,          */
/*  ratio6_1 = blue 600 rpm.                                                 */
/*                                                                           */
/*  All drive motors must be reversed correctly: the robot should drive      */
/*  forward when every drive motor spins forward.                            */
/*---------------------------------------------------------------------------*/

// Add your motors and sensors here:



/*---------------------------------------------------------------------------*/
/*  2. CHASSIS                                                               */
/*                                                                           */
/*  Follow the instructions below to enter the physical constants for your   */
/*  drivetrain. JAR-Template's docs explain each one in detail:              */
/*  https://jacksonarearobotics.github.io/JAR-Template/                      */
/*---------------------------------------------------------------------------*/

Drive chassis(

//Pick your drive setup from the list below:
//ZERO_TRACKER_NO_ODOM
//ZERO_TRACKER_ODOM
//TANK_ONE_FORWARD_ENCODER
//TANK_ONE_FORWARD_ROTATION
//TANK_ONE_SIDEWAYS_ENCODER
//TANK_ONE_SIDEWAYS_ROTATION
//TANK_TWO_ENCODER
//TANK_TWO_ROTATION
//HOLONOMIC_TWO_ENCODER
//HOLONOMIC_TWO_ROTATION
//
//Write it here:
ZERO_TRACKER_NO_ODOM,

//Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
//Use the motor names you created in section 1.

//Left Motors:
motor_group(),

//Right Motors:
motor_group(),

//Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
PORT1,

//Input your wheel diameter. (4" omnis are actually closer to 4.125"):
3.25,

//External ratio, must be in decimal, in the format of input teeth/output teeth.
//If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
//If the motor drives the wheel directly, this value is 1:
0.6,

//Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
//For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
360,

/*---------------------------------------------------------------------------*/
/*                                  PAUSE!                                   */
/*                                                                           */
/*  The rest of the drive constructor is for MECANUM / X-DRIVE robots and    */
/*  robots using POSITION TRACKING. If that isn't you, leave the rest of     */
/*  the values as they are.                                                  */
/*---------------------------------------------------------------------------*/

//If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.

//FOR MECANUM AND X-DRIVES ONLY: Input your drive motors by position, as ports. Put a minus sign in front of reversed motors.
//LF:      //RF:
PORT1,     -PORT2,

//LB:      //RB:
PORT3,     -PORT4,

//If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
//If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
//If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
3,

//Input the Forward Tracker diameter (reverse it to make the direction switch):
2.75,

//Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
//For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
//This distance is in inches:
-2,

//Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
1,

//Sideways tracker diameter (reverse to make the direction switch):
-2.75,

//Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
5.5

);

/*---------------------------------------------------------------------------*/
/*  3. MECHANISMS                                                            */
/*                                                                           */
/*  Build each mechanism from a block. Examples (see docs/GUIDE.md):         */
/*                                                                           */
/*    hh::Spinner    intake(IntakeMotor);                                    */
/*    hh::Positioner lift(LiftMotor, 12.0 / 60.0);                           */
/*    hh::Velocity   flywheel(FlywheelMotor, 3600);                          */
/*    hh::Pneumatic  mogo_clamp(Brain.ThreeWirePort.A);                      */
/*---------------------------------------------------------------------------*/

// Add your mechanisms here:



/*---------------------------------------------------------------------------*/
/*  4. DRIVER CONTROL SETTINGS AND BUTTONS                                   */
/*---------------------------------------------------------------------------*/

void configure_robot() {
  // What kind of drivetrain is it?  hh::TANK, hh::MECANUM or hh::X_DRIVE
  hh::driver::drive_type = hh::TANK;

  // How do the joysticks drive it?
  //   hh::TANK_STICKS     left stick = left side, right stick = right side
  //   hh::SPLIT_ARCADE    left stick = forward/back, right stick = turn
  //   hh::LEFT_ARCADE     left stick does both
  //   hh::RIGHT_ARCADE    right stick does both
  //   hh::FLIPPED_ARCADE  right stick = forward/back, left stick = turn
  //   hh::HOLONOMIC       left stick = move any direction, right stick = turn (mecanum / X only)
  //   hh::FIELD_CENTRIC   like HOLONOMIC, but "up" is always away from the driver
  hh::driver::style = hh::SPLIT_ARCADE;

  // Optional tuning. These are the defaults; see docs/GUIDE.md for what each does.
  hh::driver::throttle_curve = 0;     // 0 = straight line. Try 2 to 5 for finer slow control.
  hh::driver::turn_curve = 0;
  hh::driver::active_brake_kp = 0;    // 0 = off. Try 0.1 to resist being pushed.
  hh::driver::overheat_temp = 55;     // Controller rumbles when a motor gets this hot (C).

  // Mechanism settings and buttons go here. Examples:
  //   intake.bind_hold(Controller.ButtonR1, Controller.ButtonR2);
  //   mogo_clamp.bind_toggle(Controller.ButtonA);
  //   lift.add_preset("down", 0).add_preset("high", 110);
  //   lift.bind_preset(Controller.ButtonUp, "high");
  //   lift.bind_preset(Controller.ButtonDown, "down");

}

void vexcodeInit( void ) {
  // nothing to initialize
}

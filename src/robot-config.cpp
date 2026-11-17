#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

/*---------------------------------------------------------------------------*/
/*                           HHCode Robot Config                             */
/*                                                                           */
/*  This is the ONE file where you describe your robot. Work through the     */
/*  four steps in order. Anything marked CHANGE ME is an example value       */
/*  that you should replace with your own robot's.                           */
/*                                                                           */
/*  Whenever you add or rename something here, do the same in                */
/*  include/robot-config.h.                                                  */
/*                                                                           */
/*  Stuck? See docs/GETTING-STARTED.md and docs/CONFIG-RECIPES.md.           */
/*---------------------------------------------------------------------------*/

// The Brain and the controller. Leave these alone.
brain  Brain;
controller Controller1 = controller(primary);

/*---------------------------------------------------------------------------*/
/*  STEP 1: Drive motors                                                     */
/*                                                                           */
/*  A motor is made like this:                                               */
/*    motor Name = motor(port, cartridge, reversed);                         */
/*                                                                           */
/*  port:      PORT1 through PORT21, whichever the motor is plugged into.    */
/*  cartridge: ratio6_1 is blue (600 rpm), ratio18_1 is green (200 rpm),     */
/*             ratio36_1 is red (100 rpm).                                   */
/*  reversed:  true or false. Set these so that every drive motor pushes     */
/*             the robot FORWARD when it spins forward. Usually one whole    */
/*             side is reversed.                                             */
/*                                                                           */
/*  Have fewer than six drive motors? Delete the lines you don't need,       */
/*  here, in robot-config.h, and in the motor_group lists in Step 2.         */
/*---------------------------------------------------------------------------*/

// CHANGE ME: ports, cartridges and reversed.
motor LeftFront   = motor(PORT1, ratio6_1, true);
motor LeftMiddle  = motor(PORT2, ratio6_1, true);
motor LeftBack    = motor(PORT3, ratio6_1, true);
motor RightFront  = motor(PORT4, ratio6_1, false);
motor RightMiddle = motor(PORT5, ratio6_1, false);
motor RightBack   = motor(PORT6, ratio6_1, false);

/*---------------------------------------------------------------------------*/
/*  STEP 2: The chassis                                                      */
/*                                                                           */
/*  Fill in each value below. The comment above each one says what it is.    */
/*  docs/CONFIG-RECIPES.md has complete, ready-to-copy versions of this      */
/*  block for tank drive and X-drive, with and without tracking wheels.      */
/*---------------------------------------------------------------------------*/

Drive chassis(

//Pick your drive setup from the list below:
//ZERO_TRACKER_NO_ODOM         Tank drive, no tracking wheels, no position tracking. START HERE.
//ZERO_TRACKER_ODOM            Tank drive, no tracking wheels, position tracking from the drive motors.
//TANK_ONE_FORWARD_ENCODER     Tank drive, one forward tracking wheel on a red 3-wire encoder.
//TANK_ONE_FORWARD_ROTATION    Tank drive, one forward tracking wheel on a rotation sensor.
//TANK_ONE_SIDEWAYS_ENCODER    Tank drive, one sideways tracking wheel on a red 3-wire encoder.
//TANK_ONE_SIDEWAYS_ROTATION   Tank drive, one sideways tracking wheel on a rotation sensor.
//TANK_TWO_ENCODER             Tank drive, forward and sideways tracking wheels on red 3-wire encoders.
//TANK_TWO_ROTATION            Tank drive, forward and sideways tracking wheels on rotation sensors.
//HOLONOMIC_TWO_ENCODER        X-drive or mecanum, two tracking wheels on red 3-wire encoders.
//HOLONOMIC_TWO_ROTATION       X-drive or mecanum, two tracking wheels on rotation sensors.
//
//Write it here:
ZERO_TRACKER_NO_ODOM,

//Left Motors: every motor on the left side of the drive, by the names from Step 1.
motor_group(LeftFront, LeftMiddle, LeftBack),

//Right Motors:
motor_group(RightFront, RightMiddle, RightBack),

//CHANGE ME: The PORT of your inertial sensor, in PORT format (i.e. "PORT10", not simply "10"):
PORT10,

//CHANGE ME: Your wheel diameter in inches. (4" omnis are actually closer to 4.125"):
3.25,

//CHANGE ME: External gear ratio, as a decimal, in the format of input teeth/output teeth.
//If your motor has a 36-tooth gear and your wheel has a 60-tooth gear, this value will be 0.6.
//If the motor drives the wheel directly, this value is 1:
0.6,

//Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
//For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
360,

/*---------------------------------------------------------------------------*/
/*                                  PAUSE!                                   */
/*                                                                           */
/*  The rest of the chassis is for X-DRIVE / MECANUM robots and robots       */
/*  using TRACKING WHEELS. If that isn't you, leave the rest of the values   */
/*  as they are and skip to Step 4.                                          */
/*---------------------------------------------------------------------------*/

//If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.

//FOR X-DRIVE AND MECANUM ONLY: Input your drive motor PORTS by position. A minus sign reverses that motor.
//(A motor on PORT1 can't be reversed this way, so don't plug a holonomic drive motor into PORT1.)
//LF:      //RF:
PORT2,     -PORT3,

//LB:      //RB:
PORT4,     -PORT5,

//If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
//If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
//If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
//An encoder uses two three-wire ports side by side, so 5 means E and F.
5,

//Input the Forward Tracker diameter (reverse it to make the direction switch):
2.75,

//Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
//For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
//This distance is in inches:
-2,

//Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
7,

//Sideways tracker diameter (reverse to make the direction switch):
-2.75,

//Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
5.5

);

/*---------------------------------------------------------------------------*/
/*  STEP 3: Mechanism motors and pistons                                     */
/*                                                                           */
/*  Motors are made the same way as in Step 1.                               */
/*                                                                           */
/*  A piston's solenoid is made like this, where A is the three-wire port    */
/*  (A through H) on the Brain that it's plugged into:                       */
/*    digital_out Name = digital_out(Brain.ThreeWirePort.A);                 */
/*                                                                           */
/*  Keep this step BELOW the chassis. The chassis sets up three-wire ports   */
/*  for tracking wheels, and pistons made after it get the final say over    */
/*  their own ports.                                                         */
/*---------------------------------------------------------------------------*/

// CHANGE ME: add, remove and rename these to match your robot.
motor IntakeMotor = motor(PORT7, ratio6_1, false);
motor LiftMotor   = motor(PORT8, ratio36_1, false);
digital_out ClawPiston = digital_out(Brain.ThreeWirePort.A);

/*---------------------------------------------------------------------------*/
/*  STEP 4: Mechanisms                                                       */
/*                                                                           */
/*  Wrap your mechanism motors and pistons in HHCode blocks. This is what    */
/*  gives you easy commands like intake.spin(), lift.go_to("high") and       */
/*  claw.toggle(). Pick the block that matches what the mechanism does:      */
/*                                                                           */
/*  hh::Spinner     just spins            roller intake, conveyor            */
/*  hh::Positioner  moves to a position   lift, arm, wrist                   */
/*  hh::Pneumatic   piston in or out      claw, clamp, wings                 */
/*                                                                           */
/*  Two motors on one mechanism? List both: motor_group(LiftLeft, LiftRight) */
/*---------------------------------------------------------------------------*/

// CHANGE ME: add, remove and rename these to match your robot.
hh::Spinner intake = hh::Spinner(motor_group(IntakeMotor));

// The 5 is the lift's gear ratio: how many times the motor turns for the
// arm to turn once. A 12-tooth gear driving a 60-tooth gear is 60/12 = 5.
// Use 1 if the motor turns the arm directly.
hh::Positioner lift = hh::Positioner(motor_group(LiftMotor), 5);

hh::Pneumatic claw = hh::Pneumatic(ClawPiston);

void vexcodeInit( void ) {
  // nothing to initialize
}

#include "vex.h"

/*---------------------------------------------------------------------------*/
/*  An example auton that uses the drive and all three mechanism blocks.     */
/*  Copy this file, rename the function, and change the steps to make your   */
/*  own. Then add it to include/autons.h and to the auton list in main.cpp.  */
/*                                                                           */
/*  Each line runs after the one above it finishes, except lift.go_to(),     */
/*  which starts the lift and moves straight on so the lift can move WHILE   */
/*  the robot drives. Use lift.wait() when you need the lift to be there     */
/*  before the next step.                                                    */
/*                                                                           */
/*  Drive commands (distances in inches, angles in degrees):                 */
/*    chassis.drive_distance(24);     forward 24 inches                      */
/*    chassis.drive_distance(-12);    backward 12 inches                     */
/*    chassis.turn_to_angle(90);      face 90 degrees. 0 is the way the      */
/*                                    robot faced when the auton started,    */
/*                                    and clockwise is positive.             */
/*---------------------------------------------------------------------------*/

void example_auton(){
  // Open the claw and raise the lift while driving up to a game object.
  claw.retract();
  lift.go_to("mid");
  chassis.drive_distance(24);

  // Make sure the lift is up, then grab.
  lift.wait();
  claw.extend();
  wait(200, msec);   // give the piston a moment to close

  // Run the intake while backing up, then stop it.
  intake.spin();
  chassis.drive_distance(-12);
  intake.stop();

  // Turn, lift high, and let go.
  chassis.turn_to_angle(90);
  lift.go_to("high");
  lift.wait();
  claw.retract();

  // Put the lift back down to finish.
  lift.go_to("down");
  lift.wait();
}

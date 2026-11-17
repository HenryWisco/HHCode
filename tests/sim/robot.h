// The robots the tests run on. Each SIM_CONFIG is a different build of the
// test program with a different drivetrain:
//
//   1  tank drive, no odometry, plus one of every mechanism block
//   2  tank drive with two tracking wheels (odometry)
//   3  mecanum drive, odometry from the drive motor encoders
//   4  X-drive, odometry from the drive motor encoders
#pragma once
#include "vex.h"
#include "sim.h"

/** Tells the simulator what the robot physically is. */
void describe_robot();

#if SIM_CONFIG == 1
extern hh::Spinner intake;
extern hh::Positioner lift;
extern hh::Velocity flywheel;
extern hh::Pneumatic mogo_clamp;
extern hh::Pneumatic wing;
extern int b_presses;
extern int y_state;
extern bool when_flag;
extern int when_fires;
const int INTAKE_PORT = vex::PORT8;
const int LIFT_PORT = vex::PORT9;
const int FLYWHEEL_PORT = vex::PORT11;
#endif

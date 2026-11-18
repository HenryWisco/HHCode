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

#if SIM_CONFIG == 2
extern hh::WallSensor front_wall;
extern hh::WallSensor left_wall;
#endif

#if SIM_CONFIG == 1
extern hh::Spinner intake;
extern hh::Positioner lift;
extern hh::Velocity flywheel;
extern hh::Pneumatic mogo_clamp;
extern hh::Pneumatic wing;
extern hh::Pneumatic ejector;
extern hh::ColorSorter sorter;
extern hh::Pneumatic pto_piston;
extern hh::Spinner winch;
extern hh::Positioner winch_arm;
extern hh::Pto pto;
const int PTO_LEFT_PORT = vex::PORT19;
const int PTO_RIGHT_PORT = vex::PORT20;
extern hh::Sequence score_high;
extern hh::Sequence back_up;
extern int sequence_cancels;
extern int endgame_presses;
extern int b_presses;
extern int y_state;
extern bool when_flag;
extern int when_fires;
const int INTAKE_PORT = vex::PORT8;
const int LIFT_PORT = vex::PORT9;
const int FLYWHEEL_PORT = vex::PORT11;
#endif

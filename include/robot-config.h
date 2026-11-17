#pragma once
using namespace vex;

/*---------------------------------------------------------------------------*/
/*  Everything you create in robot-config.cpp also needs one line here,      */
/*  starting with the word "extern". That's what lets your other files       */
/*  (main.cpp, autons) see it.                                               */
/*                                                                           */
/*  The line here is the same as the one in robot-config.cpp, but with       */
/*  "extern" in front and everything from the = sign on removed.             */
/*---------------------------------------------------------------------------*/

class Drive;
namespace hh { class Spinner; class Positioner; class Pneumatic; }

extern brain Brain;
extern controller Controller1;

// Drive motors
extern motor LeftFront;
extern motor LeftMiddle;
extern motor LeftBack;
extern motor RightFront;
extern motor RightMiddle;
extern motor RightBack;

// Mechanism motors and pistons
extern motor IntakeMotor;
extern motor LiftMotor;
extern digital_out ClawPiston;

// The chassis and mechanisms
extern Drive chassis;
extern hh::Spinner intake;
extern hh::Positioner lift;
extern hh::Pneumatic claw;

void  vexcodeInit( void );

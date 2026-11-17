using namespace vex;

class Drive;

extern brain Brain;
extern controller Controller;
extern competition Competition;
extern Drive chassis;

// Everything you create in robot-config.cpp also needs a line here so the rest
// of the code (autons, subsystems) can use it. It's the same line with "extern"
// in front and without the part in brackets:
//
//   robot-config.cpp:  motor LeftFront = motor(PORT1, ratio6_1, false);
//   robot-config.h:    extern motor LeftFront;
//
//   robot-config.cpp:  hh::Spinner intake(IntakeMotor);
//   robot-config.h:    extern hh::Spinner intake;

// Add your devices and mechanisms below:



void vexcodeInit(void);
void configure_robot();

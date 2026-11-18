// Takes the place of src/robot-config.cpp in the test builds. See robot.h.
#include "robot.h"

using namespace vex;

brain Brain;
controller Controller(primary);

void vexcodeInit(void) {}

#if SIM_CONFIG == 1 || SIM_CONFIG == 2
/*---------------------------------------------------------------------------*/
/*  Tank drive: four blue motors, 3.25" wheels geared 36:60. The left side   */
/*  is mounted mirrored, so its motors are reversed.                         */
/*---------------------------------------------------------------------------*/

#if SIM_CONFIG == 1
// The long JAR-Template way.
motor LeftA = motor(PORT1, ratio6_1, true);
motor LeftB = motor(PORT2, ratio6_1, true);
motor RightA = motor(PORT3, ratio6_1, false);
motor RightB = motor(PORT4, ratio6_1, false);

Drive chassis(
  ZERO_TRACKER_NO_ODOM,
  motor_group(LeftA, LeftB), motor_group(RightA, RightB),
  PORT10, 3.25, 0.6, 360,
  PORT15, -PORT16, PORT17, -PORT18,
  PORT12, 2.75, -2,
  PORT13, 2.75, 5.5);
#else
// The short hh::Chassis way. 360 rpm on blue motors is the same 0.6 ratio.
Drive chassis = hh::Chassis::tank({-1, -2}, {3, 4}, 10, 3.25, ratio6_1, 360)
                  .forward_tracker(12, 2.75, -2)
                  .sideways_tracker(13, 2.75, 5.5);
#endif

void describe_robot() {
  sim::Robot& r = sim::robot;
  r.kind = sim::SIM_TANK;
  r.left_ports = {PORT1, PORT2};
  r.right_ports = {PORT3, PORT4};
  r.direction[PORT1] = -1;
  r.direction[PORT2] = -1;
  r.wheel_diameter = 3.25;
  r.wheel_ratio = 0.6;
  r.track_width = 12;
  r.forward_tracker_port = PORT12;
  r.forward_tracker_offset = -2;
  r.sideways_tracker_port = PORT13;
  r.sideways_tracker_offset = 5.5;
  r.tracker_diameter = 2.75;
  for (int port = PORT1; port <= PORT4; port++) sim::motor(port).cartridge = ratio6_1;
#if SIM_CONFIG == 1
  sim::motor(INTAKE_PORT).cartridge = ratio6_1;
  sim::motor(LIFT_PORT).cartridge = ratio36_1;
  sim::motor(FLYWHEEL_PORT).cartridge = ratio6_1;
#endif
}

#if SIM_CONFIG == 1
motor IntakeMotor = motor(PORT8, ratio6_1, false);
motor LiftMotor = motor(PORT9, ratio36_1, false);
motor FlywheelMotor = motor(PORT11, ratio6_1, false);

hh::Spinner intake(IntakeMotor);
hh::Positioner lift(LiftMotor, 12.0 / 60.0);
hh::Velocity flywheel(FlywheelMotor, 3600);
hh::Pneumatic mogo_clamp(Brain.ThreeWirePort.A);
hh::Pneumatic wing(Brain.ThreeWirePort.B);
hh::Pneumatic ejector(Brain.ThreeWirePort.C);
optical RingSensor = optical(PORT14);
hh::ColorSorter sorter(RingSensor);

// A PTO: one more motor per side that can leave the drive to run a winch.
motor PtoLeft = motor(PORT19, ratio6_1, true);
motor PtoRight = motor(PORT20, ratio6_1, false);
hh::Pneumatic pto_piston(Brain.ThreeWirePort.D);
hh::Spinner winch(PtoLeft, PtoRight);
hh::Pto pto(PtoLeft, PtoRight);

int b_presses = 0;
int y_state = 0;
bool when_flag = false;
int when_fires = 0;
#endif

void configure_robot() {
  hh::driver::drive_type = hh::TANK;
  hh::driver::style = hh::SPLIT_ARCADE;
#if SIM_CONFIG == 1
  intake.bind_hold(Controller.ButtonR1, Controller.ButtonR2);
  mogo_clamp.bind_toggle(Controller.ButtonA);
  wing.bind_hold(hh::shifted(Controller.ButtonL2, Controller.ButtonR1));
  lift.add_preset("down", 0).add_preset("high", 110);
  lift.bind_preset(Controller.ButtonUp, "high");
  lift.bind_preset(Controller.ButtonDown, "down");
  lift.bind_manual(Controller.ButtonL1, Controller.ButtonLeft);
  flywheel.bind_toggle(Controller.ButtonX, 2400);
  sorter.eject_with(ejector);
  sorter.bind_toggle(Controller.ButtonRight);
  pto.use_piston(pto_piston);
  pto.controls(winch);
  pto.bind_toggle(hh::shifted(Controller.ButtonL2, Controller.ButtonA));
  hh::on_press(Controller.ButtonB, [] { b_presses++; });
  hh::toggle(Controller.ButtonY, [] { y_state = 1; }, [] { y_state = 2; });
  hh::when([] { return when_flag; }, [] { when_fires++; });
#endif
}

#elif SIM_CONFIG == 3 || SIM_CONFIG == 4
/*---------------------------------------------------------------------------*/
/*  Holonomic drives. 3: mecanum, green motors, 4" wheels, direct drive.     */
/*  4: X-drive, blue motors, 3.25" wheels geared 1:2. The right side is      */
/*  mounted mirrored. No tracking wheels: odometry comes from the motors.    */
/*---------------------------------------------------------------------------*/

#if SIM_CONFIG == 3
const gearSetting CARTRIDGE = ratio18_1;
const double WHEEL_DIAMETER = 4;
const double WHEEL_RATIO = 1;
#else
const gearSetting CARTRIDGE = ratio6_1;
const double WHEEL_DIAMETER = 3.25;
const double WHEEL_RATIO = 0.5;
#endif

#if SIM_CONFIG == 3
// The long JAR-Template way.
motor FrontLeft = motor(PORT1, CARTRIDGE, false);
motor FrontRight = motor(PORT2, CARTRIDGE, true);
motor BackLeft = motor(PORT3, CARTRIDGE, false);
motor BackRight = motor(PORT4, CARTRIDGE, true);

Drive chassis(
  HOLONOMIC_ZERO_TRACKER_ODOM,
  motor_group(FrontLeft, BackLeft), motor_group(FrontRight, BackRight),
  PORT10, WHEEL_DIAMETER, WHEEL_RATIO, 360,
  PORT1, -PORT2, PORT3, -PORT4,
  PORT12, 2.75, 0,
  PORT13, 2.75, 0);
#else
// The short hh::Chassis way. It sets the drive type and the cartridge too.
Drive chassis = hh::Chassis::x_drive(1, -2, 3, -4, 10, 3.25, ratio6_1, 300);
#endif

void describe_robot() {
  sim::Robot& r = sim::robot;
  r.kind = SIM_CONFIG == 3 ? sim::SIM_MECANUM : sim::SIM_X_DRIVE;
  r.lf = PORT1;
  r.rf = PORT2;
  r.lb = PORT3;
  r.rb = PORT4;
  r.direction[PORT2] = -1;
  r.direction[PORT4] = -1;
  r.wheel_diameter = WHEEL_DIAMETER;
  r.wheel_ratio = WHEEL_RATIO;
  r.turn_radius = 8;
  for (int port = PORT1; port <= PORT4; port++) sim::motor(port).cartridge = CARTRIDGE;
}

void configure_robot() {
#if SIM_CONFIG == 3
  hh::driver::drive_type = hh::MECANUM;
#endif
  hh::driver::style = hh::HOLONOMIC;
}

#else
#error "Build with -DSIM_CONFIG=1, 2, 3 or 4"
#endif

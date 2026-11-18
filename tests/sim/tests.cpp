// Test scenarios. Each TEST runs in its own process on a freshly booted
// simulated robot, so they can't affect each other.
//
//   sim_tank              runs every test and prints a summary
//   sim_tank drive_test   runs one test
#include "robot.h"
#include <string>
#include <vector>

#undef main
int robot_main();   // main() from src/main.cpp, renamed by the build

/*---------------------------------------------------------------------------*/
/*  A very small test framework                                              */
/*---------------------------------------------------------------------------*/

struct Test {
  const char* name;
  void (*run)();
};

static std::vector<Test>& all_tests() {
  static std::vector<Test> list;
  return list;
}

struct Registrar {
  Registrar(const char* name, void (*run)()) { all_tests().push_back({name, run}); }
};

static int failures = 0;

#define TEST(name)                                  \
  static void name();                               \
  static Registrar registrar_##name(#name, name);   \
  static void name()

#define CHECK(condition)                                                    \
  do {                                                                      \
    if (!(condition)) {                                                     \
      printf("    FAIL line %d: %s\n", __LINE__, #condition);               \
      failures++;                                                           \
    }                                                                       \
  } while (0)

#define CHECK_NEAR(value, expected, tolerance)                              \
  do {                                                                      \
    double value_ = (value), expected_ = (expected);                        \
    if (!(fabs(value_ - expected_) <= (tolerance))) {                       \
      printf("    FAIL line %d: %s is %.3f, expected %.3f +/- %.3f\n",      \
             __LINE__, #value, value_, expected_, (double)(tolerance));     \
      failures++;                                                           \
    }                                                                       \
  } while (0)

/*---------------------------------------------------------------------------*/
/*  Helpers                                                                  */
/*---------------------------------------------------------------------------*/

using sim::robot;

// Starts the program and waits for the inertial sensor to calibrate.
static void boot(bool field = false) {
  sim::boot(field, robot_main);
  sim::run(3000);
}

// Starts the program on a competition switch and enables autonomous without
// running the selected auton, so the test itself can call chassis functions.
static void boot_for_auton() {
  boot(true);
  sim::set_mode(sim::AUTONOMOUS, false);
}

static void select_auton(const char* name) {
  for (int i = 0; i < hh::selector::count(); i++) {
    if (strcmp(hh::selector::selected_name(), name) == 0) return;
    hh::selector::next();
  }
  CHECK(strcmp(hh::selector::selected_name(), name) == 0);
}

static bool contains(const std::string& text, const char* part) { return text.find(part) != std::string::npos; }

// The robot's true heading as an error from `target`, in -180..180.
static double heading_off(double target) { return reduce_negative_180_to_180(robot.heading_deg - target); }

static double distance_to(double x, double y) { return hypot(robot.x - x, robot.y - y); }

static double speed() { return hypot(robot.forward_speed, robot.strafe_speed); }

/*---------------------------------------------------------------------------*/
/*  Every drivetrain                                                         */
/*---------------------------------------------------------------------------*/

TEST(boot_calibrates_then_shows_selector) {
  sim::boot(false, robot_main);
  sim::run(500);
  CHECK(sim::brain_shows("Calibrating"));
  sim::run(2500);
  CHECK(!sim::brain_shows("Calibrating"));
  CHECK(sim::brain_shows("Drive Test"));
  CHECK(sim::brain_shows("Auton 1 of 9"));
  CHECK(contains(sim::controller_line(1), "Drive Test"));
}

TEST(selector_touch_changes_auton_and_saves) {
  boot();
  sim::tap_screen(400, 120);
  CHECK(strcmp(hh::selector::selected_name(), "Turn Test") == 0);
  sim::run(100);
  CHECK(sim::brain_shows("Turn Test"));
  CHECK(sim::sd_files()["hh_auton.txt"] == "Turn Test");
  sim::tap_screen(100, 120);
  sim::tap_screen(100, 120);
  CHECK(strcmp(hh::selector::selected_name(), "Async Test") == 0);
}

TEST(selector_restores_choice_from_sd_card) {
  sim::sd_files()["hh_auton.txt"] = "Full Test";
  boot();
  CHECK(strcmp(hh::selector::selected_name(), "Full Test") == 0);
}

TEST(selector_works_without_sd_card) {
  sim::sd_inserted = false;
  boot();
  sim::tap_screen(400, 120);
  CHECK(strcmp(hh::selector::selected_name(), "Turn Test") == 0);
  CHECK(sim::brain_shows("no SD"));
}

TEST(selector_controller_arrows_only_while_disabled_on_field) {
  boot(true);
  sim::tap("Right");
  CHECK(strcmp(hh::selector::selected_name(), "Turn Test") == 0);
  CHECK(contains(sim::controller_line(1), "Turn Test"));
  sim::tap("Left");
  CHECK(strcmp(hh::selector::selected_name(), "Drive Test") == 0);
  sim::set_mode(sim::DRIVER);
  sim::run(100);
  sim::tap("Right");
  CHECK(strcmp(hh::selector::selected_name(), "Drive Test") == 0);
}

TEST(robot_does_not_move_while_disabled) {
  boot(true);
  sim::set_axis(3, 100);
  sim::run(1000);
  CHECK_NEAR(speed(), 0, 0.01);
  CHECK_NEAR(distance_to(0, 0), 0, 0.01);
}

TEST(drive_distance_is_accurate) {
  boot_for_auton();
  double start = sim::now();
  chassis.drive_distance(24);
  CHECK(sim::now() - start < 3000);
  CHECK_NEAR(robot.y, 24, 1.5);
  CHECK_NEAR(robot.x, 0, 1);
  CHECK_NEAR(heading_off(0), 0, 2);
  chassis.drive_distance(-24);
  CHECK_NEAR(robot.y, 0, 1.5);
}

TEST(turn_to_angle_is_accurate) {
  boot_for_auton();
  double start = sim::now();
  chassis.turn_to_angle(90);
  CHECK(sim::now() - start < 3000);
  CHECK_NEAR(heading_off(90), 0, 2);
  chassis.turn_to_angle(225);
  CHECK_NEAR(heading_off(225), 0, 2);
  chassis.turn_to_angle(0);
  CHECK_NEAR(heading_off(0), 0, 2);
  CHECK(distance_to(0, 0) < 2);
}

TEST(drive_test_auton_returns_to_start) {
  boot(true);
  select_auton("Drive Test");
  sim::set_mode(sim::AUTONOMOUS);
  double farthest = 0;
  for (int i = 0; i < 600; i++) {
    sim::run(20);
    if (robot.y > farthest) farthest = robot.y;
  }
  CHECK_NEAR(farthest, 36, 3);
  CHECK_NEAR(robot.y, 0, 2.5);
  CHECK_NEAR(heading_off(0), 0, 3);
}

TEST(full_match_flow) {
  boot(true);
  select_auton("Turn Test");
  sim::set_mode(sim::AUTONOMOUS);
  sim::run(1500);
  CHECK(fabs(heading_off(0)) > 3);          // it is turning
  sim::run(13500);
  CHECK_NEAR(heading_off(0), 0, 3);
  sim::set_mode(sim::DISABLED);
  sim::run(1000);
  CHECK_NEAR(speed(), 0, 0.5);
  sim::set_mode(sim::DRIVER);
  sim::set_axis(3, 100);
  sim::run(1000);
  CHECK(robot.forward_speed > 20);
  sim::set_axis(3, 0);
  sim::run(1000);
  double before = robot.heading_deg;
  sim::set_axis(1, 60);
  sim::run(500);
  CHECK(robot.heading_deg - before > 20);   // right stick right turns right
}

TEST(async_returns_at_once_and_wait_until_works) {
  boot_for_auton();
  double start = sim::now();
  chassis.async().drive_distance(36);
  CHECK(sim::now() - start < 1);
  CHECK(chassis.is_moving());
  chassis.wait_until(12);
  CHECK(robot.y >= 11.5 && robot.y < 16);
  CHECK(chassis.is_moving());
  chassis.wait();
  CHECK(!chassis.is_moving());
  CHECK_NEAR(robot.y, 36, 1.5);
}

TEST(wait_until_measures_degrees_for_turns) {
  boot_for_auton();
  chassis.async().turn_to_angle(120);
  chassis.wait_until(45);
  CHECK(robot.heading_deg >= 44 && robot.heading_deg < 70);
  chassis.wait();
  CHECK_NEAR(heading_off(120), 0, 2);
}

TEST(async_test_auton_runs) {
  boot_for_auton();
  async_test();
  CHECK(sim::brain_shows("Passed 12 inches"));
  CHECK(distance_to(0, 0) < 5);
  CHECK_NEAR(heading_off(0), 0, 3);
}

TEST(new_motion_replaces_background_motion) {
  boot_for_auton();
  chassis.async().drive_distance(60);
  sim::run(300);
  chassis.turn_to_angle(90);
  CHECK(!chassis.is_moving());
  CHECK_NEAR(heading_off(90), 0, 3);
  sim::run(300);
  CHECK(speed() < 2);
}

TEST(cancel_motion_stops_the_robot) {
  boot_for_auton();
  chassis.async().drive_distance(60);
  sim::run(400);
  CHECK(robot.forward_speed > 10);
  chassis.cancel_motion();
  CHECK(!chassis.is_moving());
  sim::run(400);
  CHECK(speed() < 1);
}

TEST(chain_exits_early_and_still_moving) {
  boot_for_auton();
  double start = sim::now();
  chassis.drive_distance(24);
  double settled_time = sim::now() - start;
  CHECK(speed() < 2);

  double from = robot.y;
  start = sim::now();
  chassis.chain().drive_distance(24);
  double chained_time = sim::now() - start;
  CHECK(chained_time < settled_time - 150);
  CHECK(robot.forward_speed > 3);
  CHECK(robot.y - from > 20 && robot.y - from < 24.5);

  // chain() only applies to one motion: the next one settles normally.
  chassis.drive_distance(6);
  CHECK(speed() < 2);
}

TEST(slew_ramps_the_voltage) {
  boot_for_auton();
  chassis.drive_slew = 30;   // volts per second
  chassis.async().drive_distance(48);
  sim::run(105);
  CHECK(chassis.DriveR.voltage(volt) > 2 && chassis.DriveR.voltage(volt) < 3.8);
  sim::run(100);
  CHECK(chassis.DriveR.voltage(volt) > 5 && chassis.DriveR.voltage(volt) < 6.8);
  chassis.wait();
  CHECK_NEAR(robot.y, 48, 1.5);

  chassis.drive_slew = 0;
  chassis.async().drive_distance(-48);
  sim::run(25);
  CHECK(chassis.DriveR.voltage(volt) < -9);   // no slew: full power at once
  chassis.wait();
}

TEST(stall_detection_gives_up_when_blocked) {
  boot_for_auton();
  chassis.stall_time = 300;
  robot.blocked = true;
  double start = sim::now();
  chassis.drive_distance(24);
  CHECK(sim::now() - start < 800);
  CHECK(chassis.interfered);

  robot.blocked = false;
  chassis.drive_distance(12);
  CHECK(!chassis.interfered);
  CHECK_NEAR(robot.y, 12, 1.5);
  chassis.turn_to_angle(90);
  CHECK(!chassis.interfered);
  CHECK_NEAR(heading_off(90), 0, 2);
}

TEST(without_stall_detection_a_blocked_drive_times_out) {
  boot_for_auton();
  robot.blocked = true;
  double start = sim::now();
  chassis.drive_distance(24);
  CHECK_NEAR(sim::now() - start, 5000, 100);
  CHECK(!chassis.interfered);
}

static void leftover_auton() { chassis.async().drive_distance(200); }

TEST(background_motion_is_cancelled_when_driver_control_starts) {
  boot(true);
  hh::selector::add("Leftover", "Leaves a background motion running", leftover_auton);
  select_auton("Leftover");
  sim::set_mode(sim::AUTONOMOUS);
  sim::run(700);
  CHECK(robot.forward_speed > 20);
  sim::set_mode(sim::DISABLED);
  sim::run(300);
  sim::set_mode(sim::DRIVER);
  sim::run(700);
  CHECK(!chassis.is_moving());
  CHECK(speed() < 1);
}

TEST(joystick_curve_shape) {
  boot();
  CHECK_NEAR(hh::driver::curve(100, 5), 100, 0.01);
  CHECK_NEAR(hh::driver::curve(-100, 5), -100, 0.01);
  CHECK_NEAR(hh::driver::curve(0, 5), 0, 0.01);
  CHECK_NEAR(hh::driver::curve(50, 0), 50, 0.01);
  CHECK(hh::driver::curve(50, 5) < 45 && hh::driver::curve(50, 5) > 20);
  CHECK_NEAR(hh::driver::curve(-50, 5), -hh::driver::curve(50, 5), 0.01);
}

TEST(tuner_adjusts_runs_and_stops) {
  boot();
  hh::tuner::enabled = true;
  sim::run(100);
  CHECK(contains(sim::controller_line(1), "Drive kP"));
  CHECK(contains(sim::controller_line(2), "1.500"));
  sim::tap("Right");
  CHECK_NEAR(chassis.drive_kp, 1.55, 0.001);
  CHECK(contains(sim::controller_line(2), "1.550"));
  sim::tap("Left");
  CHECK_NEAR(chassis.drive_kp, 1.5, 0.001);
  sim::tap("Down");
  CHECK(contains(sim::controller_line(1), "Drive kI"));
  sim::tap("Up");
  sim::tap("Up");
  CHECK(contains(sim::controller_line(1), "Turn slew"));

  sim::tap("A");                       // runs Drive Test
  sim::run(500);
  CHECK(distance_to(0, 0) > 2);
  CHECK(contains(sim::controller_line(3), "RUNNING"));
  sim::set_axis(3, -100);              // sticks are ignored during a run
  sim::run(200);
  CHECK(robot.forward_speed > 0);
  sim::set_axis(3, 0);
  sim::tap("B");
  sim::run(600);
  CHECK(speed() < 1);
  CHECK(contains(sim::controller_line(3), "A:run"));

  double from = robot.y;
  sim::tap("A");                       // this time let it finish
  sim::run(12000);
  CHECK_NEAR(robot.y, from, 2.5);
  CHECK(contains(sim::controller_line(3), "A:run"));

  sim::set_axis(3, 100);               // and the sticks drive again
  sim::run(500);
  CHECK(robot.forward_speed > 20);
}

TEST(ports_page_and_unplugged_warning) {
  sim::devices[vex::PORT1] = kDeviceTypeMotorSensor;
  sim::devices[vex::PORT5] = kDeviceTypeMotorSensor;
  sim::devices[vex::PORT10] = kDeviceTypeImuSensor;
  sim::motor(vex::PORT5).temperature = 61;
  boot();
  CHECK(sim::brain_shows("Ports>"));
  CHECK(!sim::brain_shows("NO IMU!"));

  sim::tap_screen(30, 15);               // "Ports>" in the title bar opens the ports page
  sim::run(600);
  CHECK(sim::brain_shows("tap to go back"));
  CHECK(sim::brain_shows("IMU"));
  CHECK(sim::brain_shows("61C"));
  CHECK(strcmp(hh::selector::selected_name(), "Drive Test") == 0);
  CHECK(strcmp(hh::ports::name(10), "IMU") == 0);
  CHECK(strcmp(hh::ports::name(7), "") == 0);
  CHECK_NEAR(hh::ports::temperature(5), 61, 0.01);
  CHECK(!hh::ports::lost(1));

  int rumbles = sim::rumbles();
  sim::devices[vex::PORT1] = kDeviceTypeNoSensor;   // a cable comes out
  sim::run(700);
  CHECK(sim::rumbles() == rumbles + 1);
  CHECK(contains(sim::controller_line(3), "PORT 1 UNPLUGGED"));
  CHECK(hh::ports::lost(1));
  CHECK(sim::brain_shows("LOST"));
  sim::run(3000);
  CHECK(sim::rumbles() == rumbles + 1);  // one warning, not one every second

  sim::tap_screen(300, 120);             // any tap goes back, without changing the auton
  sim::run(200);
  CHECK(sim::brain_shows("Auton 1 of"));
  CHECK(strcmp(hh::selector::selected_name(), "Drive Test") == 0);
}

TEST(missing_inertial_sensor_is_flagged) {
  sim::imu_installed = false;
  boot();
  CHECK(sim::brain_shows("NO IMU!"));
}

TEST(forced_turn_direction) {
  boot_for_auton();
  chassis.clockwise().turn_to_angle(270);        // the short way would be 90 left
  CHECK_NEAR(robot.heading_deg, 270, 3);
  chassis.turn_to_angle(0);                      // only the one turn is forced
  CHECK_NEAR(robot.heading_deg, 360, 3);
  chassis.counterclockwise().turn_to_angle(90);  // the short way would be 90 right
  CHECK_NEAR(robot.heading_deg, 90, 3);
  chassis.clockwise().turn_to_angle(180);        // already the short way: no change
  CHECK_NEAR(robot.heading_deg, 180, 3);
  chassis.clockwise().turn_to_angle(180);        // already there: doesn't spin a full circle
  CHECK_NEAR(robot.heading_deg, 180, 3);

  chassis.async().counterclockwise().turn_to_angle(270);   // works in the background too
  chassis.wait();
  CHECK_NEAR(robot.heading_deg, -90, 3);
}

TEST(backward_drive_constants) {
  boot_for_auton();
  chassis.set_drive_backward_constants(4, 1.5, 0, 10, 0);
  chassis.async().drive_distance(24);
  sim::run(200);
  CHECK_NEAR(chassis.DriveR.voltage(volt), 10, 0.5);     // forward is unchanged
  chassis.wait();
  chassis.async().drive_distance(-24);
  sim::run(200);
  CHECK_NEAR(chassis.DriveR.voltage(volt), -4, 0.5);     // backward is capped at 4 volts
  chassis.wait();
  CHECK_NEAR(robot.y, 0, 1.5);
  chassis.clear_drive_backward_constants();
  chassis.async().drive_distance(-24);
  sim::run(200);
  CHECK_NEAR(chassis.DriveR.voltage(volt), -10, 0.5);
  chassis.wait();
}

// Puts the simulated robot back at the origin, facing forward.
static void back_to_origin() {
  robot.x = 0;
  robot.y = 0;
  robot.heading_deg = 0;
}

static void mirror_routine() {
  chassis.drive_distance(12);
  chassis.turn_to_angle(60);
  chassis.drive_distance(10);
}

TEST(mirrored_auton_is_a_mirror_image) {
  boot(true);
  hh::selector::add_mirrored("Side A", "Side B", "Drives, turns right, drives", mirror_routine);
  select_auton("Side A");
  sim::set_mode(sim::AUTONOMOUS);
  sim::run(8000);
  double x = robot.x, y = robot.y, heading = robot.heading_deg;
  CHECK(x > 5);
  CHECK_NEAR(heading, 60, 3);
  CHECK(!chassis.is_mirrored());

  sim::set_mode(sim::DISABLED);
  sim::run(500);
  back_to_origin();
  select_auton("Side B");
  sim::set_mode(sim::AUTONOMOUS);
  sim::run(200);
  CHECK(hh::selector::mirrored());
  sim::run(7800);
  CHECK_NEAR(robot.x, -x, 1);
  CHECK_NEAR(robot.y, y, 1);
  CHECK_NEAR(robot.heading_deg, -heading, 3);
  CHECK(!chassis.is_mirrored());           // back to normal once the auton ends
}

TEST(mirroring_is_cleared_when_driver_control_starts) {
  boot(true);
  hh::selector::add_mirrored("Side A", "Side B", "Leaves a motion running", leftover_auton);
  select_auton("Side B");
  sim::set_mode(sim::AUTONOMOUS);
  sim::run(500);                           // cut off mid-auton, like a real match
  CHECK(chassis.is_mirrored());
  sim::set_mode(sim::DISABLED);
  sim::run(300);
  sim::set_mode(sim::DRIVER);
  sim::run(300);
  CHECK(!chassis.is_mirrored());
}

TEST(graph_draws_the_latest_motion) {
  boot_for_auton();
  hh::graph::enabled = true;
  CHECK(!hh::graph::owns_screen());
  chassis.turn_to_angle(90);
  double heading_at_end = heading_off(90);
  sim::run(400);
  CHECK(hh::graph::owns_screen());
  CHECK(sim::brain_shows("Turn"));
  CHECK(sim::brain_shows("tap for autons"));
  CHECK(sim::brain_lines() > 20);       // -1 would mean something was drawn off the screen
  CHECK_NEAR(hh::graph::start_error(), 90, 1);
  CHECK_NEAR(hh::graph::final_error(), -heading_at_end, 0.5);
  CHECK(hh::graph::duration_ms() > 300 && hh::graph::duration_ms() < 3000);
  CHECK(hh::graph::overshoot() >= 0 && hh::graph::overshoot() < 10);

  chassis.drive_distance(12);           // a new motion replaces the graph
  sim::run(400);
  CHECK(sim::brain_shows("Drive"));
  CHECK_NEAR(hh::graph::start_error(), 12, 0.5);

  sim::set_mode(sim::DISABLED);
  sim::tap_screen(400, 120);            // a tap gives the screen back to the selector...
  sim::run(200);
  CHECK(!hh::graph::owns_screen());
  CHECK(sim::brain_shows("Auton 1 of"));
  CHECK(strcmp(hh::selector::selected_name(), "Drive Test") == 0);   // ...without changing the auton
}

TEST(graph_fits_a_long_motion) {
  boot_for_auton();
  hh::graph::enabled = true;
  robot.blocked = true;
  chassis.drive_distance(24);           // runs for the full 5 second timeout
  sim::run(400);
  CHECK(hh::graph::samples() > 100 && hh::graph::samples() <= 230);
  CHECK_NEAR(hh::graph::duration_ms(), 5000, 100);
  CHECK(sim::brain_lines() > 100);
  CHECK_NEAR(hh::graph::final_error(), 24, 0.1);
  CHECK_NEAR(hh::graph::overshoot(), 0, 0.01);
}

TEST(graph_is_off_by_default) {
  boot_for_auton();
  chassis.turn_to_angle(90);
  sim::run(400);
  CHECK(!hh::graph::owns_screen());
  CHECK(hh::graph::samples() == 0);
  CHECK(sim::brain_lines() == 0);
}

#if SIM_CONFIG == 1 || SIM_CONFIG == 2
/*---------------------------------------------------------------------------*/
/*  Tank drive                                                              */
/*---------------------------------------------------------------------------*/

TEST(mirrored_swings_swap_sides) {
  boot_for_auton();
  chassis.left_swing_to_angle(90);
  chassis.right_swing_to_angle(30);
  double x = robot.x, y = robot.y;
  CHECK_NEAR(heading_off(30), 0, 3);

  sim::run(500);
  back_to_origin();
  chassis.set_mirrored(true);
  chassis.left_swing_to_angle(90);
  CHECK_NEAR(heading_off(-90), 0, 3);
  CHECK_NEAR(chassis.get_absolute_heading(), 90, 3);   // the auton still sees 90
  chassis.right_swing_to_angle(30);
  CHECK_NEAR(heading_off(-30), 0, 3);
  CHECK_NEAR(robot.x, -x, 1.5);
  CHECK_NEAR(robot.y, y, 1.5);
}

TEST(swing_turns) {
  boot_for_auton();
  chassis.left_swing_to_angle(90);
  CHECK_NEAR(heading_off(90), 0, 3);
  chassis.right_swing_to_angle(0);
  CHECK_NEAR(heading_off(0), 0, 3);
  CHECK(distance_to(0, 0) > 4);   // an S shape ends somewhere else
}

TEST(full_test_auton_ends_near_start) {
  boot_for_auton();
  full_test();
  CHECK(distance_to(0, 0) < 14);
  CHECK_NEAR(heading_off(0), 0, 3);
}

TEST(split_arcade_sticks) {
  boot();
  sim::set_axis(3, 50);
  sim::run(100);
  CHECK_NEAR(chassis.DriveL.voltage(volt), 6, 0.1);
  CHECK_NEAR(chassis.DriveR.voltage(volt), 6, 0.1);
  sim::run(900);
  CHECK(robot.forward_speed > 10);
  CHECK_NEAR(heading_off(0), 0, 1);
  sim::set_axis(1, 50);
  sim::run(100);
  CHECK_NEAR(chassis.DriveL.voltage(volt), 12, 0.1);
  CHECK_NEAR(chassis.DriveR.voltage(volt), 0, 0.1);
  // Full forward and full turn: scaled so neither side passes 12 volts.
  sim::set_axis(3, 100);
  sim::set_axis(1, 100);
  sim::run(100);
  CHECK_NEAR(chassis.DriveL.voltage(volt), 12, 0.1);
  CHECK_NEAR(chassis.DriveR.voltage(volt), 0, 0.1);
}

TEST(tank_sticks_and_deadband) {
  boot();
  hh::driver::style = hh::TANK_STICKS;
  sim::set_axis(3, 80);
  sim::set_axis(2, -40);
  sim::run(100);
  CHECK_NEAR(chassis.DriveL.voltage(volt), 9.6, 0.1);
  CHECK_NEAR(chassis.DriveR.voltage(volt), -4.8, 0.1);
  sim::set_axis(3, 4);
  sim::set_axis(2, -4);
  sim::run(100);
  CHECK_NEAR(chassis.DriveL.voltage(volt), 0, 0.01);
  CHECK_NEAR(chassis.DriveR.voltage(volt), 0, 0.01);
}

TEST(other_arcade_styles) {
  boot();
  hh::driver::style = hh::LEFT_ARCADE;
  sim::set_axis(3, 50);
  sim::set_axis(4, 25);
  sim::run(100);
  CHECK_NEAR(chassis.DriveL.voltage(volt), 9, 0.1);
  CHECK_NEAR(chassis.DriveR.voltage(volt), 3, 0.1);
  sim::set_axis(3, 0);
  sim::set_axis(4, 0);
  hh::driver::style = hh::RIGHT_ARCADE;
  sim::set_axis(2, 50);
  sim::set_axis(1, -25);
  sim::run(100);
  CHECK_NEAR(chassis.DriveL.voltage(volt), 3, 0.1);
  CHECK_NEAR(chassis.DriveR.voltage(volt), 9, 0.1);
  sim::set_axis(1, 0);
  hh::driver::style = hh::FLIPPED_ARCADE;
  sim::set_axis(4, 25);
  sim::run(100);
  CHECK_NEAR(chassis.DriveL.voltage(volt), 9, 0.1);
  CHECK_NEAR(chassis.DriveR.voltage(volt), 3, 0.1);
  hh::driver::turn_scale = 0.5;
  sim::run(100);
  CHECK_NEAR(chassis.DriveL.voltage(volt), 7.5, 0.1);
  CHECK_NEAR(chassis.DriveR.voltage(volt), 4.5, 0.1);
}

TEST(active_brake_pushes_back) {
  boot();
  hh::driver::active_brake_kp = 0.1;
  sim::run(100);
  CHECK_NEAR(chassis.DriveR.voltage(volt), 0, 0.1);
  sim::motor(PORT3).position_deg += 60;   // someone shoves the right side forward
  sim::run(25);
  CHECK(chassis.DriveR.voltage(volt) < -3);
  CHECK(fabs(chassis.DriveL.voltage(volt)) < 1);
  sim::run(1500);                         // and it works its way back
  CHECK(fabs(chassis.DriveR.voltage(volt)) < 1.5);
  sim::set_axis(3, 50);                   // the sticks still win
  sim::run(100);
  CHECK_NEAR(chassis.DriveR.voltage(volt), 6, 0.1);
}
#endif

#if SIM_CONFIG == 1
/*---------------------------------------------------------------------------*/
/*  Mechanism blocks and buttons                                             */
/*---------------------------------------------------------------------------*/

TEST(intake_hold_buttons) {
  boot();
  sim::set_button("R1", true);
  sim::run(100);
  CHECK(intake.is_spinning());
  CHECK_NEAR(sim::motor(INTAKE_PORT).volts, 12, 0.01);
  sim::run(500);
  CHECK(sim::motor(INTAKE_PORT).velocity_dps > 2000);
  sim::set_button("R1", false);
  sim::run(100);
  CHECK(!intake.is_spinning());
  CHECK(sim::motor(INTAKE_PORT).command == sim::MotorState::STOPPED);
  sim::set_button("R2", true);
  sim::run(100);
  CHECK_NEAR(sim::motor(INTAKE_PORT).volts, -12, 0.01);
  sim::set_button("R2", false);
  sim::run(100);
  CHECK(!intake.is_spinning());
}

TEST(intake_speed_and_auton_commands) {
  boot_for_auton();
  intake.spin(60);
  CHECK_NEAR(sim::motor(INTAKE_PORT).volts, 7.2, 0.01);
  intake.speed = 50;
  intake.reverse();
  CHECK_NEAR(sim::motor(INTAKE_PORT).volts, -6, 0.01);
  CHECK_NEAR(intake.commanded(), -50, 0.01);
  intake.velocity_mode = true;
  intake.spin();
  CHECK(sim::motor(INTAKE_PORT).command == sim::MotorState::VELOCITY);
  CHECK_NEAR(sim::motor(INTAKE_PORT).velocity_pct, 50, 0.01);
  intake.stop();
  CHECK(sim::motor(INTAKE_PORT).command == sim::MotorState::STOPPED);
}

TEST(shift_key_combo) {
  boot();
  sim::set_button("L2", true);
  sim::run(60);
  sim::set_button("R1", true);
  sim::run(100);
  CHECK(wing.is_extended());
  CHECK(sim::digital(1));
  CHECK(!intake.is_spinning());       // plain R1 doesn't fire while L2 is held
  sim::set_button("R1", false);
  sim::run(100);
  CHECK(!wing.is_extended());
  sim::set_button("L2", false);
  sim::run(60);
  sim::set_button("R1", true);
  sim::run(100);
  CHECK(intake.is_spinning());
  CHECK(!wing.is_extended());
}

TEST(piston_toggle_pulse_and_reversed) {
  boot();
  CHECK(!sim::digital(0));
  sim::tap("A");
  CHECK(sim::digital(0));
  sim::tap("A");
  CHECK(!sim::digital(0));
  mogo_clamp.pulse(200);
  CHECK(sim::digital(0));
  sim::run(150);
  CHECK(sim::digital(0));
  sim::run(100);
  CHECK(!sim::digital(0));
  mogo_clamp.reversed = true;
  mogo_clamp.extend();
  CHECK(mogo_clamp.is_extended());
  CHECK(!sim::digital(0));
  mogo_clamp.retract();
  CHECK(sim::digital(0));
}

TEST(on_press_toggle_and_when) {
  boot();
  sim::tap("B");
  sim::tap("B");
  sim::set_button("B", true);
  sim::run(500);                // holding counts once
  sim::set_button("B", false);
  sim::run(60);
  CHECK(b_presses == 3);
  sim::tap("Y");
  CHECK(y_state == 1);
  sim::tap("Y");
  CHECK(y_state == 2);

  when_flag = true;
  sim::run(100);
  CHECK(when_fires == 1);       // once per false -> true
  when_flag = false;
  sim::run(50);
  when_flag = true;
  sim::run(50);
  CHECK(when_fires == 2);
}

TEST(jam_protection_backs_out_and_resumes) {
  boot_for_auton();
  intake.jam_protection = true;
  intake.spin();
  sim::run(600);
  CHECK(intake.jam_count() == 0);       // spinning up is not a jam
  sim::motor(INTAKE_PORT).jammed = true;
  sim::run(350);
  CHECK(intake.is_unjamming());
  CHECK_NEAR(sim::motor(INTAKE_PORT).volts, -12, 0.01);
  sim::motor(INTAKE_PORT).jammed = false;
  sim::run(300);
  CHECK(!intake.is_unjamming());
  CHECK_NEAR(sim::motor(INTAKE_PORT).volts, 12, 0.01);
  CHECK(intake.jam_count() == 1);
  sim::run(1000);
  CHECK(intake.jam_count() == 1);
}

TEST(mechanisms_forget_commands_when_disabled) {
  boot_for_auton();
  intake.spin();
  flywheel.set_rpm(2400);
  sim::run(500);
  sim::set_mode(sim::DISABLED);
  sim::run(500);
  CHECK(!intake.is_spinning());
  CHECK(!flywheel.is_spinning());
  sim::set_mode(sim::AUTONOMOUS, false);
  sim::run(1000);
  CHECK(sim::motor(INTAKE_PORT).command == sim::MotorState::STOPPED);
  CHECK(fabs(sim::motor(FLYWHEEL_PORT).velocity_dps) < 50);
}

TEST(intake_stops_when_driver_control_starts) {
  boot(true);
  sim::set_mode(sim::DRIVER);
  sim::run(100);
  intake.spin();
  sim::set_mode(sim::DISABLED);
  sim::run(400);
  sim::set_mode(sim::DRIVER);
  sim::run(100);
  CHECK(!intake.is_spinning());
}

TEST(lift_presets_from_buttons) {
  boot();
  sim::tap("Up");
  sim::run(2500);
  CHECK_NEAR(lift.position(), 110, 2.5);
  CHECK(lift.is_settled());
  sim::tap("Down");
  sim::run(300);
  CHECK(!lift.is_settled());
  sim::run(2200);
  CHECK_NEAR(lift.position(), 0, 2.5);
}

TEST(lift_go_to_returns_at_once_and_wait_blocks) {
  boot_for_auton();
  double start = sim::now();
  lift.go_to(60);
  CHECK(sim::now() - start < 1);
  lift.wait();
  CHECK(sim::now() - start > 300 && sim::now() - start < 2500);
  CHECK_NEAR(lift.position(), 60, 2.5);
  lift.go_to("high");
  CHECK_NEAR(lift.target(), 110, 0.01);
  lift.go_to("no such preset");        // prints a message, keeps the old target
  CHECK_NEAR(lift.target(), 110, 0.01);
}

TEST(lift_soft_limits) {
  boot_for_auton();
  lift.set_limits(0, 90);
  lift.go_to(200);
  CHECK_NEAR(lift.target(), 90, 0.01);
  lift.move(12);                       // manual drive stops at the limit too
  sim::run(3000);
  CHECK(lift.position() < 95);
  lift.move(-12);
  sim::run(3000);
  CHECK(lift.position() > -5);
}

TEST(lift_manual_then_holds) {
  boot();
  sim::set_button("L1", true);
  sim::run(400);
  CHECK(lift.position() > 10);
  sim::set_button("L1", false);
  sim::run(60);
  double held = lift.target();
  sim::run(800);
  CHECK_NEAR(lift.position(), held, 3);
  sim::set_button("Left", true);
  sim::run(300);
  CHECK(lift.position() < held - 5);
}

TEST(lift_gravity_feedforward) {
  boot_for_auton();
  sim::motor(LIFT_PORT).load_volts = 2;    // a heavy lift
  lift.go_to(60);
  sim::run(3000);
  CHECK(60 - lift.position() > 2);         // sags without kg
  lift.kg = 2;
  sim::run(2000);
  CHECK_NEAR(lift.position(), 60, 2);
}

TEST(lift_zero_keeps_it_still) {
  boot_for_auton();
  lift.go_to(50);
  sim::run(2000);
  lift.zero();
  CHECK_NEAR(lift.position(), 0, 0.01);
  sim::run(1000);
  CHECK_NEAR(lift.position(), 0, 2.5);
}

TEST(flywheel_reaches_speed) {
  boot_for_auton();
  CHECK(!flywheel.ready());
  flywheel.set_rpm(2400);
  sim::run(100);
  CHECK(!flywheel.ready());
  double start = sim::now();
  flywheel.wait_until_ready();
  CHECK(sim::now() - start < 2000);
  CHECK(flywheel.ready());
  CHECK_NEAR(flywheel.rpm(), 2400, 108);
  flywheel.stop();
  CHECK(!flywheel.ready());
  sim::run(3000);
  CHECK(fabs(flywheel.rpm()) < 50);
}

TEST(flywheel_toggle_button) {
  boot();
  sim::tap("X");
  CHECK_NEAR(flywheel.target_rpm(), 2400, 0.01);
  sim::run(1500);
  CHECK(flywheel.ready());
  sim::tap("X");
  CHECK(!flywheel.is_spinning());
}

TEST(overheat_warning) {
  boot();
  CHECK(sim::rumbles() == 0);
  sim::motor(INTAKE_PORT).temperature = 60;
  sim::run(1500);
  CHECK(sim::rumbles() == 1);
  CHECK(contains(sim::controller_line(3), "HOT"));
  CHECK(contains(sim::controller_line(3), "Spinner"));
  sim::run(5000);
  CHECK(sim::rumbles() == 1);          // doesn't nag every second
}

// A game piece of this hue passes the sensor for `ms`, then there is a gap.
static void show_piece(double hue, double ms) {
  sim::optical_hue = hue;
  sim::optical_near = true;
  sim::run(ms);
  sim::optical_near = false;
  sim::run(300);
}

TEST(sorter_throws_out_the_wrong_color) {
  boot_for_auton();
  CHECK(sim::optical_light);
  sorter.keep(hh::RED);
  show_piece(10, 100);                  // red: kept
  CHECK(sorter.rejected() == 0);
  CHECK(!sim::digital(2));

  sim::optical_hue = 220;               // blue arrives and sits in front of the sensor
  sim::optical_near = true;
  sim::run(60);
  CHECK(sorter.seen() == hh::BLUE);
  CHECK(sorter.is_ejecting());
  CHECK(sim::digital(2));
  sim::run(400);
  CHECK(!sim::digital(2));
  CHECK(sorter.rejected() == 1);        // one eject per piece, however long it stays
  sim::optical_near = false;
  sim::run(100);

  show_piece(220, 100);
  CHECK(sorter.rejected() == 2);
  sorter.keep(hh::BLUE);                // other alliance
  show_piece(220, 100);
  CHECK(sorter.rejected() == 2);
  show_piece(350, 100);                 // red hue wraps around 0
  CHECK(sorter.rejected() == 3);
}

TEST(sorter_ignores_other_colors_and_far_objects) {
  boot_for_auton();
  sorter.keep(hh::RED);
  sim::optical_hue = 220;               // blue, but nothing is close
  sim::run(300);
  CHECK(sorter.rejected() == 0);
  show_piece(100, 200);                 // green
  CHECK(sorter.rejected() == 0);
  sim::optical_hue = 220;               // a single flicker of blue is not enough
  sim::optical_near = true;
  sim::run(10);
  sim::optical_near = false;
  sim::run(300);
  CHECK(sorter.rejected() == 0);
  sorter.use_proximity = false;
  sim::run(100);
  CHECK(sorter.rejected() == 1);
}

TEST(sorter_delay_off_and_toggle_button) {
  boot();
  sorter.eject_delay = 200;
  sorter.keep(hh::RED);
  sim::optical_hue = 220;
  sim::optical_near = true;
  sim::run(120);
  CHECK(!sim::digital(2));              // waiting for the piece to reach the ejector
  sim::run(150);
  CHECK(sim::digital(2));
  sim::run(200);
  CHECK(!sim::digital(2));
  sim::optical_near = false;
  sim::run(100);

  sim::tap("Right");                    // paused from the controller
  CHECK(!sorter.is_active());
  show_piece(220, 100);
  CHECK(sorter.rejected() == 1);
  sim::tap("Right");
  CHECK(sorter.is_active());
  sorter.keep(hh::NO_COLOR);            // or turned off in code
  show_piece(220, 100);
  CHECK(sorter.rejected() == 1);
}

TEST(sorter_ejects_with_the_intake) {
  boot_for_auton();
  sorter.eject_with(intake, -100);
  sorter.keep(hh::RED);
  intake.spin();
  sim::run(300);
  sim::optical_hue = 220;
  sim::optical_near = true;
  sim::run(60);
  CHECK_NEAR(sim::motor(INTAKE_PORT).volts, -12, 0.01);
  CHECK_NEAR(intake.commanded(), 100, 0.01);   // it still knows what it was asked to do
  sim::optical_near = false;
  sim::run(250);
  CHECK_NEAR(sim::motor(INTAKE_PORT).volts, 12, 0.01);
  CHECK(!sim::digital(2));

  sim::optical_near = true;             // stopping during an eject really stops
  sim::run(60);
  intake.stop();
  sim::run(300);
  CHECK(sim::motor(INTAKE_PORT).command == sim::MotorState::STOPPED);
}

TEST(tuner_pauses_button_bindings) {
  boot();
  hh::tuner::enabled = true;
  sim::set_button("R1", true);
  sim::run(200);
  CHECK(!intake.is_spinning());
}
#endif

#if SIM_CONFIG == 2
/*---------------------------------------------------------------------------*/
/*  Tank odometry and path following                                         */
/*---------------------------------------------------------------------------*/

static void start_odom() {
  boot_for_auton();
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  sim::run(50);
}

TEST(odometry_tracks_the_true_position) {
  start_odom();
  chassis.drive_distance(24);
  chassis.turn_to_angle(90);
  chassis.drive_distance(12);
  chassis.turn_to_angle(200);
  chassis.drive_distance(-10);
  CHECK_NEAR(chassis.get_X_position(), robot.x, 0.5);
  CHECK_NEAR(chassis.get_Y_position(), robot.y, 0.5);
}

TEST(tank_odom_test_auton_returns_to_start) {
  boot_for_auton();
  tank_odom_test();
  CHECK(distance_to(0, 0) < 4);
  CHECK_NEAR(heading_off(0), 0, 3);
}

TEST(turn_and_drive_to_point) {
  start_odom();
  chassis.turn_to_point(24, 24);
  CHECK_NEAR(heading_off(45), 0, 3);
  chassis.drive_to_point(24, 24);
  CHECK(distance_to(24, 24) < 3.5);
  chassis.drive_to_point(0, 0);         // backs up to get there
  CHECK(distance_to(0, 0) < 3.5);
}

TEST(drive_to_pose_arrives_facing_the_right_way) {
  start_odom();
  chassis.drive_to_pose(24, 24, 90);
  CHECK(distance_to(24, 24) < 5);
  CHECK_NEAR(heading_off(90), 0, 20);
}

// Distance from the robot to the nearest point on a path.
static double distance_to_path(const std::vector<path_point>& path) {
  double best = 1e9;
  for (size_t i = 0; i + 1 < path.size(); i++) {
    double dx = path[i + 1].x - path[i].x, dy = path[i + 1].y - path[i].y;
    double length_squared = dx * dx + dy * dy;
    double t = length_squared == 0 ? 0 : ((robot.x - path[i].x) * dx + (robot.y - path[i].y) * dy) / length_squared;
    t = t < 0 ? 0 : (t > 1 ? 1 : t);
    double d = hypot(robot.x - (path[i].x + t * dx), robot.y - (path[i].y + t * dy));
    if (d < best) best = d;
  }
  return best;
}

TEST(follow_path_through_a_curve) {
  start_odom();
  std::vector<path_point> path = {{0, 0}, {0, 24}, {24, 48}, {48, 48}};
  double start = sim::now();
  chassis.async().follow_path({{0, 24}, {24, 48}, {48, 48}});
  double worst = 0;
  double slowest_midway = 1e9;
  while (chassis.is_moving() && sim::now() - start < 15000) {
    sim::run(20);
    double off = distance_to_path(path);
    if (off > worst) worst = off;
    if (robot.y > 10 && robot.x < 40 && speed() < slowest_midway) slowest_midway = speed();
  }
  CHECK(sim::now() - start < 8000);
  CHECK(distance_to(48, 48) < 4);
  CHECK(worst < 8);                     // corners are rounded, but not by much
  CHECK(slowest_midway > 5);            // it never stops at a waypoint
  CHECK_NEAR(heading_off(90), 0, 25);
}

TEST(follow_path_with_points_closer_than_lookahead) {
  start_odom();
  std::vector<path_point> points;
  for (int y = 4; y <= 48; y += 4) points.push_back({0, (float)y});
  double start = sim::now();
  chassis.follow_path(points, 12);
  CHECK(sim::now() - start < 5000);
  CHECK(distance_to(0, 48) < 4);
}

TEST(follow_path_backwards) {
  start_odom();
  double farthest_forward = 0;
  chassis.async().follow_path({{0, -24}, {-24, -48}}, 10, true);
  double start = sim::now();
  while (chassis.is_moving() && sim::now() - start < 15000) {
    sim::run(20);
    if (robot.y > farthest_forward) farthest_forward = robot.y;
  }
  CHECK(sim::now() - start < 8000);
  CHECK(distance_to(-24, -48) < 4);
  CHECK(farthest_forward < 2);
  CHECK_NEAR(heading_off(45), 0, 30);   // still facing away from where it went
}

TEST(follow_path_wait_until_counts_inches) {
  start_odom();
  chassis.async().follow_path({{0, 24}, {24, 48}, {48, 48}});
  chassis.wait_until(20);
  CHECK(chassis.is_moving());
  CHECK(distance_to(0, 0) >= 19.5 && distance_to(0, 0) < 26);
  chassis.wait();
}

TEST(follow_path_single_point_and_empty_path) {
  start_odom();
  chassis.follow_path({{0, 20}});
  CHECK(distance_to(0, 20) < 4);
  chassis.async().follow_path({});      // nothing to do, and must not leave async() armed
  CHECK(!chassis.is_moving());
  double from = robot.y;
  chassis.drive_distance(6);            // so this must block until it has driven
  CHECK(!chassis.is_moving());
  CHECK(robot.y - from > 3);
}

// Follows a path with one sharp corner at (0, 30) and returns how close the
// robot came to that corner.
static double closest_to_corner(float smoothing) {
  back_to_origin();
  chassis.set_coordinates(0, 0, 0);
  sim::run(50);
  chassis.path_smoothing = smoothing;
  chassis.async().follow_path({{0, 30}, {30, 30}}, 6);
  double closest = 1e9;
  double start = sim::now();
  while (chassis.is_moving() && sim::now() - start < 15000) {
    sim::run(20);
    if (distance_to(0, 30) < closest) closest = distance_to(0, 30);
  }
  CHECK(distance_to(30, 30) < 4);
  chassis.drive_stop(brake);
  sim::run(500);
  return closest;
}

TEST(path_smoothing_rounds_corners) {
  boot_for_auton();
  odom_constants();
  double sharp = closest_to_corner(0);
  double rounded = closest_to_corner(0.9);
  CHECK(rounded > sharp + 1.5);

  std::vector<path_point> smooth = chassis.smooth_path({{0, 0}, {0, 30}, {30, 30}}, 0.9);
  CHECK(smooth.size() > 15);
  CHECK_NEAR(smooth.front().x, 0, 0.001);       // the ends don't move
  CHECK_NEAR(smooth.front().y, 0, 0.001);
  CHECK_NEAR(smooth.back().x, 30, 0.001);
  CHECK_NEAR(smooth.back().y, 30, 0.001);
}

TEST(mirrored_turn_direction) {
  boot_for_auton();
  chassis.set_mirrored(true);
  chassis.clockwise().turn_to_angle(270);       // clockwise for the auton is left for the robot
  CHECK_NEAR(robot.heading_deg, -270, 3);
}

TEST(mirrored_odometry_and_paths) {
  boot_for_auton();
  odom_constants();
  chassis.set_mirrored(true);
  chassis.set_coordinates(0, 0, 0);
  sim::run(50);
  chassis.turn_to_point(24, 24);
  CHECK_NEAR(heading_off(-45), 0, 3);
  chassis.drive_to_point(24, 24);
  CHECK(distance_to(-24, 24) < 3.5);
  chassis.follow_path({{24, 48}, {48, 48}});
  CHECK(distance_to(-48, 48) < 4);
  chassis.turn_to_angle(200);
  chassis.drive_distance(-10);
  // The auton's own coordinates are the un-mirrored ones.
  CHECK_NEAR(chassis.get_X_position(), -robot.x, 0.75);
  CHECK_NEAR(chassis.get_Y_position(), robot.y, 0.75);
}

TEST(chained_point_to_point) {
  start_odom();
  chassis.chain().drive_to_point(0, 24);
  CHECK(speed() > 3);
  chassis.drive_to_point(24, 36);
  CHECK(distance_to(24, 36) < 3.5);
}
#endif

#if SIM_CONFIG == 3 || SIM_CONFIG == 4
/*---------------------------------------------------------------------------*/
/*  Mecanum and X-drive                                                      */
/*---------------------------------------------------------------------------*/

TEST(strafe_distance_is_accurate) {
  boot_for_auton();
  chassis.strafe_distance(24);
  CHECK_NEAR(robot.x, 24, 1.5);
  CHECK_NEAR(robot.y, 0, 1.5);
  CHECK_NEAR(heading_off(0), 0, 2);
  chassis.strafe_distance(-24);
  CHECK_NEAR(robot.x, 0, 1.5);
}

TEST(strafe_test_auton_returns_to_start) {
  boot_for_auton();
  strafe_test();
  CHECK(distance_to(0, 0) < 2.5);
}

TEST(strafe_holds_the_heading_asked_for) {
  boot_for_auton();
  chassis.strafe_distance(24, 30);
  CHECK_NEAR(heading_off(30), 0, 4);
}

TEST(strafe_async_wait_until) {
  boot_for_auton();
  chassis.async().strafe_distance(24);
  chassis.wait_until(10);
  CHECK(chassis.is_moving());
  CHECK(robot.x >= 9.5 && robot.x < 15);
  chassis.wait();
  CHECK_NEAR(robot.x, 24, 1.5);
}

TEST(strafe_stall_detection) {
  boot_for_auton();
  chassis.stall_time = 300;
  chassis.strafe_distance(24);          // moving freely is not a stall
  CHECK(!chassis.interfered);
  CHECK_NEAR(robot.x, 24, 1.5);
  robot.blocked = true;
  double start = sim::now();
  chassis.strafe_distance(24);
  CHECK(sim::now() - start < 800);
  CHECK(chassis.interfered);
}

TEST(encoder_odometry_tracks_the_true_position) {
  boot_for_auton();
  chassis.set_coordinates(0, 0, 0);
  sim::run(50);
  chassis.drive_distance(20);
  chassis.strafe_distance(15);
  chassis.turn_to_angle(90);
  chassis.strafe_distance(-10);
  chassis.drive_distance(-8);
  CHECK_NEAR(chassis.get_X_position(), robot.x, 0.75);
  CHECK_NEAR(chassis.get_Y_position(), robot.y, 0.75);
}

TEST(holonomic_odom_test_auton_returns_to_start) {
  boot_for_auton();
  holonomic_odom_test();
  CHECK(distance_to(0, 0) < 4);
  CHECK_NEAR(heading_off(0), 0, 5);
  CHECK_NEAR(chassis.get_X_position(), robot.x, 1.5);
  CHECK_NEAR(chassis.get_Y_position(), robot.y, 1.5);
}

TEST(mirrored_strafe_and_holonomic_pose) {
  boot_for_auton();
  odom_constants();
  chassis.set_mirrored(true);
  chassis.set_coordinates(0, 0, 0);
  sim::run(50);
  chassis.strafe_distance(12);
  CHECK_NEAR(robot.x, -12, 2);
  CHECK_NEAR(robot.y, 0, 1.5);
  chassis.holonomic_drive_to_pose(18, 18, 90);
  CHECK(distance_to(-18, 18) < 4);
  CHECK_NEAR(heading_off(-90), 0, 5);
  CHECK_NEAR(chassis.get_X_position(), -robot.x, 1.5);
  CHECK_NEAR(chassis.get_Y_position(), robot.y, 1.5);
}

TEST(holonomic_sticks) {
  boot();
  sim::set_axis(4, 100);                // left stick right: strafe right
  sim::run(800);
  CHECK(robot.strafe_speed > 15);
  CHECK(fabs(robot.forward_speed) < 2);
  CHECK_NEAR(heading_off(0), 0, 2);
  sim::set_axis(4, 0);
  sim::set_axis(3, 100);                // left stick up: forward
  sim::run(800);
  CHECK(robot.forward_speed > 15);
  CHECK(fabs(robot.strafe_speed) < 2);
  sim::set_axis(3, 0);
  sim::run(800);
  double before = robot.heading_deg;
  sim::set_axis(1, 50);                 // right stick right: turn right
  sim::run(400);
  CHECK(robot.heading_deg - before > 15);
}

TEST(field_centric_sticks) {
  boot();
  hh::driver::style = hh::FIELD_CENTRIC;
  sim::set_axis(1, 50);                 // turn to face roughly right
  while (robot.heading_deg < 90) sim::run(20);
  sim::set_axis(1, 0);
  sim::run(800);
  double x = robot.x, y = robot.y;
  sim::set_axis(3, 100);                // stick up still means away from the driver
  sim::run(800);
  CHECK(robot.y - y > 8);
  CHECK(fabs(robot.x - x) < (robot.y - y) * 0.25);
  sim::set_axis(3, 0);
  sim::run(800);

  hh::driver::reset_field_centric();    // now "away" is wherever the robot points
  x = robot.x;
  y = robot.y;
  double facing = robot.heading_deg * 3.14159265 / 180.0;
  sim::set_axis(3, 100);
  sim::run(800);
  double along = (robot.x - x) * sin(facing) + (robot.y - y) * cos(facing);
  CHECK(along > 8);
  CHECK_NEAR(hypot(robot.x - x, robot.y - y), along, 1.5);
}
#endif

#if SIM_CONFIG == 3
TEST(strafe_multiplier_corrects_wheel_slip) {
  boot_for_auton();
  robot.strafe_efficiency = 0.85;       // mecanum rollers slip sideways
  chassis.strafe_distance(24);
  CHECK_NEAR(robot.x, 24 * 0.85, 1.5);  // comes up short...
  chassis.strafe_multiplier = 0.85;
  double from = robot.x;
  chassis.strafe_distance(24);
  CHECK_NEAR(robot.x - from, 24, 1.5);  // ...until the multiplier is set
}
#endif

#if SIM_CONFIG == 4
TEST(x_drive_distances_allow_for_the_45_degree_wheels) {
  boot_for_auton();
  chassis.drive_distance(24);
  CHECK_NEAR(robot.y, 24, 1.5);
  // The motors turned 24 / sqrt(2) inches worth, not 24.
  double wheel_inches = chassis.DriveLF.position(deg) * 3.14159265 * 3.25 * 0.5 / 360.0;
  CHECK_NEAR(wheel_inches, 24 / sqrt(2.0), 1.5);
}
#endif

/*---------------------------------------------------------------------------*/
/*  Odometry map, hh::Chassis and PTO                                        */
/*---------------------------------------------------------------------------*/

TEST(odom_map_opens_from_the_title_bar) {
  boot();
  CHECK(sim::brain_shows("Map>"));
  sim::tap_screen(110, 15);              // "Map>" in the title bar
  sim::run(400);
  CHECK(sim::brain_shows("Odometry map"));
  CHECK(sim::brain_shows("H "));
#if SIM_CONFIG == 1
  CHECK(sim::brain_shows("No odometry"));
#else
  CHECK(sim::brain_shows("X "));
  CHECK(sim::brain_lines() >= 13);       // the grid and the robot arrow, all on the screen
#endif
  CHECK(strcmp(hh::selector::selected_name(), "Drive Test") == 0);

  sim::tap_screen(300, 120);             // any tap goes back, without changing the auton
  sim::run(200);
  CHECK(sim::brain_shows("Auton 1 of"));
  CHECK(strcmp(hh::selector::selected_name(), "Drive Test") == 0);

  sim::tap_screen(30, 15);               // "Ports>" still opens the ports page
  sim::run(600);
  CHECK(sim::brain_shows("Ports  (tap to go back)"));
}

TEST(chassis_builder_picks_the_right_setup) {
  boot();
  Drive plain = hh::Chassis::tank({1, -2}, {-3, 4}, 10, 4, ratio18_1, 200);
  CHECK(plain.drive_setup == ZERO_TRACKER_NO_ODOM);
  Drive motor_odom = hh::Chassis::tank({1, -2}, {-3, 4}, 10, 4, ratio18_1, 200).odom(12);
  CHECK(motor_odom.drive_setup == ZERO_TRACKER_ODOM);
  Drive one_forward = hh::Chassis::tank({1}, {-3}, 10, 4, ratio18_1, 200).forward_encoder('C', 2.75, 1);
  CHECK(one_forward.drive_setup == TANK_ONE_FORWARD_ENCODER);
  Drive one_sideways = hh::Chassis::tank({1}, {-3}, 10, 4, ratio18_1, 200).odom(12).sideways_tracker(-7, 2, 3);
  CHECK(one_sideways.drive_setup == TANK_ONE_SIDEWAYS_ROTATION);
  Drive two = hh::Chassis::tank({1}, {-3}, 10, 4, ratio18_1, 200).forward_tracker(6, 2, 1).sideways_tracker(7, 2, 3);
  CHECK(two.drive_setup == TANK_TWO_ROTATION);
  Drive mecanum = hh::Chassis::mecanum(1, -2, 3, -4, 10, 4, ratio18_1, 200);
  CHECK(mecanum.drive_setup == HOLONOMIC_ZERO_TRACKER_ODOM);
  Drive tracked = hh::Chassis::mecanum(1, -2, 3, -4, 10, 4, ratio18_1, 200)
                    .forward_encoder('A', 2.75, 1).sideways_encoder('C', 2.75, 3);
  CHECK(tracked.drive_setup == HOLONOMIC_TWO_ENCODER);
}

#if SIM_CONFIG != 1
TEST(odom_map_draws_a_trail_and_stays_up_in_auton) {
  boot(true);
  sim::tap_screen(110, 15);
  sim::run(300);
  sim::set_mode(sim::AUTONOMOUS, false);
  chassis.drive_distance(24);
  sim::run(300);
  CHECK(sim::brain_shows("Odometry map"));   // not cleared when the auton started
  CHECK(hh::odom_map::trail_points() >= 8);
  CHECK(sim::brain_lines() >= 13 + 6);       // grid, arrow and trail, all on the screen

  hh::odom_map::center_x = 500;              // the robot is nowhere near what the map shows
  sim::run(300);
  CHECK(sim::brain_shows("off the map"));
  CHECK(sim::brain_lines() == 10);           // only the grid: nothing is drawn outside the map

  hh::odom_map::clear_trail();
  CHECK(hh::odom_map::trail_points() == 0);
}
#endif

#if SIM_CONFIG == 2
TEST(chassis_builder_tank_with_trackers) {
  boot_for_auton();
  CHECK(chassis.drive_setup == TANK_TWO_ROTATION);
  CHECK(hh::driver::drive_type == hh::TANK);
  chassis.drive_distance(24);
  CHECK_NEAR(robot.y, 24, 1);
  CHECK_NEAR(chassis.get_Y_position(), 24, 1);
  chassis.turn_to_angle(90);
  CHECK_NEAR(heading_off(90), 0, 2);
}
#endif

#if SIM_CONFIG == 4
TEST(chassis_builder_sets_the_drive_type) {
  boot_for_auton();
  CHECK(chassis.drive_setup == HOLONOMIC_ZERO_TRACKER_ODOM);
  CHECK(hh::driver::drive_type == hh::X_DRIVE);
}
#endif

#if SIM_CONFIG == 1
TEST(pto_motors_drive_with_their_side_until_shifted) {
  boot();
  CHECK(pto.is_driving());
  CHECK(sim::digital(3));                // piston on port D: extended = on the drive
  sim::set_mode(sim::DRIVER);
  sim::set_axis(3, 100);
  sim::run(200);
  CHECK(fabs(sim::motor(vex::PORT1).volts) > 6);
  CHECK_NEAR(sim::motor(PTO_LEFT_PORT).volts, sim::motor(vex::PORT1).volts, 0.01);
  CHECK_NEAR(sim::motor(PTO_RIGHT_PORT).volts, sim::motor(vex::PORT3).volts, 0.01);

  winch.spin();                          // ignored: its motors are busy driving
  sim::run(50);
  CHECK(!winch.is_spinning());
  CHECK_NEAR(sim::motor(PTO_LEFT_PORT).volts, sim::motor(vex::PORT1).volts, 0.01);

  pto.to_mechanism();
  sim::run(50);
  CHECK(!pto.is_driving());
  CHECK(!sim::digital(3));
  CHECK(sim::motor(PTO_LEFT_PORT).command == sim::MotorState::STOPPED);
  CHECK(sim::motor(PTO_RIGHT_PORT).command == sim::MotorState::STOPPED);
  CHECK(sim::motor(vex::PORT1).command == sim::MotorState::VOLTAGE);   // the rest still drive

  winch.spin();
  sim::run(50);
  CHECK(winch.is_spinning());
  CHECK(sim::motor(PTO_LEFT_PORT).command == sim::MotorState::VOLTAGE);
  CHECK_NEAR(fabs(sim::motor(PTO_LEFT_PORT).volts), 12, 0.01);
  CHECK_NEAR(fabs(sim::motor(PTO_RIGHT_PORT).volts), 12, 0.01);

  pto.to_drive();                        // the winch lets go and the motors rejoin the drive
  sim::run(50);
  CHECK(!winch.is_spinning());
  CHECK(sim::digital(3));
  CHECK_NEAR(sim::motor(PTO_LEFT_PORT).volts, sim::motor(vex::PORT1).volts, 0.01);
  CHECK_NEAR(sim::motor(PTO_RIGHT_PORT).volts, sim::motor(vex::PORT3).volts, 0.01);
}

TEST(pto_button_toggles_without_firing_the_plain_button) {
  boot();
  sim::set_mode(sim::DRIVER);
  sim::run(50);
  bool clamp_before = sim::digital(0);
  sim::set_button("L2", true);
  sim::tap("A");
  sim::set_button("L2", false);
  CHECK(!pto.is_driving());
  CHECK(sim::digital(0) == clamp_before);   // plain A (the clamp) didn't fire
  sim::set_button("L2", true);
  sim::tap("A");
  sim::set_button("L2", false);
  CHECK(pto.is_driving());
}

TEST(pto_motors_follow_chassis_motions) {
  boot_for_auton();
  chassis.async().drive_distance(24);
  sim::run(150);
  CHECK(fabs(sim::motor(PTO_LEFT_PORT).volts) > 3);
  CHECK_NEAR(sim::motor(PTO_LEFT_PORT).volts, sim::motor(vex::PORT1).volts, 0.01);
  CHECK_NEAR(sim::motor(PTO_RIGHT_PORT).volts, sim::motor(vex::PORT3).volts, 0.01);
  chassis.wait();
  chassis.drive_stop(brake);
  sim::run(20);
  CHECK(sim::motor(PTO_LEFT_PORT).command == sim::MotorState::STOPPED);
  CHECK(sim::motor(PTO_LEFT_PORT).stopping == vex::brakeType::brake);

  chassis.async().left_swing_to_angle(90);   // left side powered, right side held
  sim::run(100);
  CHECK(sim::motor(PTO_LEFT_PORT).command == sim::MotorState::VOLTAGE);
  CHECK(sim::motor(PTO_RIGHT_PORT).command == sim::MotorState::STOPPED);
  CHECK(sim::motor(PTO_RIGHT_PORT).stopping == vex::brakeType::hold);
  chassis.cancel_motion();

  pto.to_mechanism();                        // now motions leave the PTO motors alone
  chassis.async().drive_distance(12);
  sim::run(150);
  CHECK(sim::motor(vex::PORT1).command == sim::MotorState::VOLTAGE);
  CHECK(sim::motor(PTO_LEFT_PORT).command == sim::MotorState::STOPPED);
  chassis.wait();
}

TEST(pto_can_start_on_the_mechanism) {
  pto.start_on_drive = false;
  boot();
  CHECK(!pto.is_driving());
  CHECK(!sim::digital(3));
  sim::set_mode(sim::DRIVER);
  sim::set_axis(3, 100);
  sim::run(200);
  CHECK(sim::motor(PTO_LEFT_PORT).command == sim::MotorState::STOPPED);
  winch.reverse();
  sim::run(50);
  CHECK(sim::motor(PTO_LEFT_PORT).command == sim::MotorState::VOLTAGE);
}
#endif

/*---------------------------------------------------------------------------*/
/*  One-button sequences, match timer, driver profiles and wall resets       */
/*---------------------------------------------------------------------------*/

#if SIM_CONFIG == 1
// Holds L2 and taps another button: the shift combos the sequences are on.
static void shift_tap(const char* name) {
  sim::set_button("L2", true);
  sim::tap(name);
  sim::set_button("L2", false);
}

TEST(sequence_runs_its_steps_in_order_while_the_driver_drives) {
  boot();
  sim::set_mode(sim::DRIVER);
  sim::run(50);
  shift_tap("B");
  CHECK(score_high.is_running());
  CHECK_NEAR(lift.target(), 110, 0.01);      // step 1: lift up
  CHECK(!intake.is_spinning());              // step 2 waits for the lift

  sim::set_axis(3, 100);                     // the driver can still drive
  sim::run(100);
  CHECK(fabs(sim::motor(vex::PORT1).volts) > 6);
  sim::set_axis(3, 0);

  // Wait for the lift to arrive: then the intake runs backwards for 400 ms.
  for (int i = 0; i < 300 && !intake.is_spinning(); i++) sim::run(10);
  CHECK(intake.commanded() < 0);
  CHECK_NEAR(lift.position(), 110, 4);
  sim::run(600);
  CHECK(!intake.is_spinning());              // last steps: stop, lift back down
  CHECK_NEAR(lift.target(), 0, 0.01);
  CHECK(!score_high.is_running());
  CHECK(sequence_cancels == 0);              // finishing is not cancelling

  shift_tap("B");                            // and it can run again
  CHECK(score_high.is_running());
  CHECK_NEAR(lift.target(), 110, 0.01);
}

TEST(sequence_second_press_cancels_it) {
  boot();
  sim::set_mode(sim::DRIVER);
  sim::run(50);
  shift_tap("B");
  for (int i = 0; i < 300 && !intake.is_spinning(); i++) sim::run(10);
  CHECK(intake.is_spinning());
  shift_tap("B");                            // cancelled in the middle of the 400 ms wait
  CHECK(!score_high.is_running());
  CHECK(sequence_cancels == 1);
  CHECK(!intake.is_spinning());              // on_cancel stopped it
  sim::run(1000);
  CHECK_NEAR(lift.target(), 110, 0.01);      // the later steps never ran
}

TEST(sequence_that_drives_has_the_chassis_until_a_stick_moves) {
  boot();
  sim::set_mode(sim::DRIVER);
  sim::run(50);
  shift_tap("Y");
  CHECK(back_up.is_running());
  sim::run(300);
  CHECK(robot.y < -1);                       // backing up with the sticks centred
  double y = robot.y;
  sim::run(200);
  CHECK(robot.y < y);

  sim::set_axis(3, 100);                     // the driver takes over
  sim::run(100);
  CHECK(!back_up.is_running());
  CHECK(sim::motor(vex::PORT3).command == sim::MotorState::VOLTAGE);
  CHECK(sim::motor(vex::PORT3).volts > 6);   // right side forward: the stick, not the sequence
  sim::set_axis(3, 0);
  sim::run(500);

  double start_y = robot.y;                  // left alone, it runs to the end
  shift_tap("Y");
  sim::run(5000);
  CHECK(!back_up.is_running());
  CHECK_NEAR(robot.y - start_y, -12, 1.5);
  CHECK_NEAR(heading_off(90), 0, 3);
  sim::set_axis(3, 100);                     // and then the sticks work again
  sim::run(200);
  CHECK(sim::motor(vex::PORT3).volts > 6);
}

TEST(sequence_stops_when_the_robot_is_disabled) {
  boot(true);
  sim::set_mode(sim::DRIVER);
  sim::run(50);
  score_high.start();
  sim::run(100);
  CHECK(score_high.is_running());
  sim::set_mode(sim::DISABLED);
  sim::run(100);
  CHECK(!score_high.is_running());
  score_high.start();                        // can't be started while disabled
  CHECK(!score_high.is_running());

  sim::set_mode(sim::AUTONOMOUS, false);     // in an auton: start, then wait for it
  score_high.start();
  score_high.wait();
  CHECK(!score_high.is_running());
  CHECK_NEAR(lift.target(), 0, 0.01);
}

TEST(match_timer_warns_and_unlocks_the_endgame) {
  boot(true);
  CHECK(!hh::match_timer::running());
  sim::set_mode(sim::DRIVER);
  sim::run(100);
  CHECK(hh::match_timer::running());
  CHECK_NEAR(hh::match_timer::time_left(), 105, 0.5);
  CHECK(!hh::match_timer::in_endgame());

  int rumbles = sim::rumbles();
  shift_tap("X");                            // too early: a short buzz and nothing else
  CHECK(endgame_presses == 0);
  CHECK(sim::rumbles() == rumbles + 1);

  rumbles = sim::rumbles();
  sim::run(73000);
  CHECK(sim::rumbles() == rumbles);          // quiet until 30 seconds are left
  sim::run(3000);
  CHECK(sim::rumbles() == rumbles + 1);
  CHECK(hh::match_timer::in_endgame());
  shift_tap("X");
  CHECK(endgame_presses == 1);
  sim::run(15000);
  CHECK(sim::rumbles() == rumbles + 2);      // 15 seconds left
  CHECK_NEAR(hh::match_timer::time_left(), 13.5, 1.5);

  sim::set_mode(sim::DISABLED);              // the match ends
  sim::run(100);
  CHECK(!hh::match_timer::running());
  CHECK(hh::match_timer::time_left() == -1);
}

TEST(match_timer_only_counts_in_practice_when_started) {
  hh::match_timer::driver_length = 60;
  hh::match_timer::show_on_controller = true;
  hh::match_timer::warn_at(50, "...");
  boot();
  sim::set_mode(sim::DRIVER);
  sim::run(80000);
  CHECK(sim::rumbles() == 0);
  CHECK(!hh::match_timer::running());
  shift_tap("X");                            // no clock: endgame mechanisms just work
  CHECK(endgame_presses == 1);

  hh::match_timer::start();
  sim::run(500);
  CHECK(hh::match_timer::running());
  CHECK(sim::rumbles() == 0);
  CHECK(contains(sim::controller_line(2), "Time 0:59"));
  sim::run(10000);
  CHECK(sim::rumbles() == 1);                // the extra warning at 50 seconds left
  sim::run(21000);
  CHECK(sim::rumbles() == 2);                // 30 left
  sim::run(31000);
  CHECK(sim::rumbles() == 3);                // 15 left, then the clock runs out
  CHECK(!hh::match_timer::running());
}
#endif

#if SIM_CONFIG == 2
TEST(driver_profiles_are_picked_on_the_brain_and_saved) {
  hh::driver::turn_scale = 0.9;
  sim::sd_files()["hh_driver.txt"] = "Sam";
  boot();
  CHECK(strcmp(hh::profiles::selected_name(), "Sam") == 0);
  CHECK(hh::driver::style == hh::TANK_STICKS);
  CHECK_NEAR(hh::driver::throttle_curve, 3, 0.001);
  CHECK_NEAR(hh::driver::turn_curve, 2, 0.001);
  CHECK(sim::brain_shows("Driver: Sam"));

  sim::tap_screen(350, 60);                  // "Driver: ..." -> the next driver
  sim::run(100);
  CHECK(strcmp(hh::profiles::selected_name(), "Jo") == 0);
  CHECK(sim::brain_shows("Driver: Jo"));
  CHECK(sim::sd_files()["hh_driver.txt"] == "Jo");
  CHECK(hh::driver::style == hh::SPLIT_ARCADE);      // Sam's layout didn't leak into Jo's
  CHECK_NEAR(hh::driver::throttle_curve, 0, 0.001);
  CHECK_NEAR(hh::driver::turn_scale, 0.5, 0.001);
  CHECK(strcmp(hh::selector::selected_name(), "Drive Test") == 0);   // the auton didn't change

  sim::tap_screen(350, 60);                  // wraps round to the first
  CHECK(strcmp(hh::profiles::selected_name(), "Alex") == 0);
  CHECK_NEAR(hh::driver::turn_scale, 0.9, 0.001);    // the settings from configure_robot()

  sim::tap_screen(400, 120);                 // the rest of the screen still picks autons
  CHECK(strcmp(hh::selector::selected_name(), "Turn Test") == 0);
  CHECK(strcmp(hh::profiles::selected_name(), "Alex") == 0);
}

TEST(driver_profiles_controller_arrows_and_their_own_curves) {
  hh::driver::curve_tuning = true;
  boot(true);
  sim::tap("Down");                          // disabled on the field: Down = next driver
  CHECK(strcmp(hh::profiles::selected_name(), "Sam") == 0);
  CHECK(contains(sim::controller_line(2), "Driver: Sam"));
  sim::tap("Up");
  CHECK(strcmp(hh::profiles::selected_name(), "Alex") == 0);
  sim::tap("Down");

  sim::set_mode(sim::DRIVER);
  sim::run(100);
  sim::tap("Down");                          // not while driving
  CHECK(strcmp(hh::profiles::selected_name(), "Sam") == 0);

  sim::tap("Right");                         // live curve tuning: throttle 3 -> 3.5
  CHECK_NEAR(hh::driver::throttle_curve, 3.5, 0.001);
  CHECK(sim::sd_files().count("hh_curves_Sam.txt") == 1);
  CHECK(sim::sd_files().count("hh_curves.txt") == 0);

  hh::profiles::select(0);                   // Alex is untouched by Sam's tuning
  CHECK_NEAR(hh::driver::throttle_curve, 0, 0.001);
  hh::profiles::select(1);                   // and Sam gets the tuned curve back
  CHECK_NEAR(hh::driver::throttle_curve, 3.5, 0.001);
  CHECK_NEAR(hh::driver::turn_curve, 2, 0.001);
}

TEST(wall_reset_corrects_drift_in_x_and_y) {
  boot_for_auton();
  CHECK_NEAR(front_wall.inches(), 72 - 6.5, 0.1);
  CHECK_NEAR(left_wall.inches(), 72 - 5.5, 0.1);

  chassis.set_coordinates(3, -4, 0);         // odometry has drifted; the robot is really at (0, 0)
  CHECK(hh::wall_reset() == 2);
  CHECK_NEAR(chassis.get_X_position(), 0, 0.1);
  CHECK_NEAR(chassis.get_Y_position(), 0, 0.1);
  CHECK(strcmp(front_wall.status, "ok") == 0);

  // Somewhere else, facing another way: the front sensor now measures X and
  // the left one Y.
  chassis.drive_distance(20);
  chassis.turn_to_angle(90);
  chassis.set_coordinates(robot.x + 2, robot.y - 3, chassis.get_absolute_heading());
  CHECK(hh::wall_reset() == 2);
  CHECK_NEAR(chassis.get_X_position(), robot.x, 0.2);
  CHECK_NEAR(chassis.get_Y_position(), robot.y, 0.2);
  CHECK_NEAR(heading_off(chassis.get_absolute_heading()), 0, 0.01);   // heading is left alone

  // Tracking carries on from the corrected position.
  chassis.drive_distance(10);
  CHECK_NEAR(chassis.get_X_position(), robot.x, 0.5);
  CHECK_NEAR(chassis.get_Y_position(), robot.y, 0.5);

  chassis.set_coordinates(robot.x - 1, robot.y, chassis.get_absolute_heading());
  CHECK(front_wall.reset());                 // one sensor on its own fixes one direction
  CHECK_NEAR(chassis.get_X_position(), robot.x, 0.2);
}

TEST(wall_reset_ignores_readings_it_cannot_trust) {
  boot_for_auton();
  chassis.set_coordinates(1, 1, 0);

  sim::block_distance(vex::PORT7, 20);       // a game piece in front of the front sensor
  CHECK(hh::wall_reset() == 1);              // only the left sensor is believed
  CHECK(strcmp(front_wall.status, "too far from expected") == 0);
  CHECK_NEAR(chassis.get_X_position(), 0, 0.1);
  CHECK_NEAR(chassis.get_Y_position(), 1, 0.1);
  sim::block_distance(vex::PORT7, -1);

  hh::walls::max_range = 30;                 // the walls are further away than that
  CHECK(hh::wall_reset() == 0);
  CHECK(strcmp(left_wall.status, "sees nothing") == 0);
  CHECK(front_wall.inches() == -1);
  hh::walls::max_range = 70;

  chassis.turn_to_angle(45);                 // pointing into a corner
  CHECK(hh::wall_reset() == 0);
  CHECK(strcmp(front_wall.status, "not square to a wall") == 0);
  CHECK_NEAR(chassis.get_Y_position(), 1, 0.3);

  chassis.turn_to_angle(10);                 // a little off square is fine, and still exact
  CHECK(hh::wall_reset() == 2);
  CHECK_NEAR(chassis.get_X_position(), robot.x, 0.2);
  CHECK_NEAR(chassis.get_Y_position(), robot.y, 0.2);
}

TEST(wall_reset_in_a_mirrored_auton_and_on_the_map) {
  boot_for_auton();
  // A mirrored auton thinks in mirror image: the robot is really 10 inches
  // left of center, so the routine sees itself 10 inches right of center.
  robot.x = -10;
  chassis.set_mirrored(true);
  hh::walls::max_correction = 0;             // the check is off: any correction is accepted
  CHECK(hh::wall_reset() == 2);
  CHECK_NEAR(chassis.get_X_position(), 10, 0.2);
  CHECK_NEAR(chassis.get_Y_position(), 0, 0.2);
  chassis.set_mirrored(false);

  sim::set_mode(sim::DISABLED);
  sim::tap_screen(110, 15);                  // the map lists what each wall sensor reads
  sim::run(400);
  CHECK(sim::brain_shows("Walls F66 L"));
}
#endif

/*---------------------------------------------------------------------------*/
/*  Runner                                                                   */
/*---------------------------------------------------------------------------*/

int main(int argc, char** argv) {
  if (argc >= 2) {
    for (Test& test : all_tests()) {
      if (strcmp(test.name, argv[1]) != 0) continue;
      describe_robot();
      test.run();
      sim::finish(failures == 0 ? 0 : 1);
    }
    printf("No test called %s\n", argv[1]);
    return 2;
  }

  int failed = 0;
  for (Test& test : all_tests()) {
    printf("  %s\n", test.name);
    fflush(stdout);
    std::string program = argv[0];
#ifdef _WIN32
    for (char& c : program) if (c == '/') c = '\\';   // cmd.exe needs backslashes
#endif
    std::string command = "\"" + program + "\" " + test.name;
    if (system(command.c_str()) != 0) {
      printf("    ^ FAILED\n");
      failed++;
    }
  }
  printf("\n%d of %d tests passed (SIM_CONFIG %d)\n", (int)all_tests().size() - failed, (int)all_tests().size(), SIM_CONFIG);
  return failed == 0 ? 0 : 1;
}

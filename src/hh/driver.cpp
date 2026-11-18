// Driver control: turns joystick positions into motor voltages.
//
// Every 20 ms usercontrol() calls update(), which runs the button bindings
// and then drive(). drive() reads the sticks for the chosen layout, works
// out how much "forward", "turn" and "strafe" the driver is asking for, and
// output() sends that to the motors.
//
// Also here: live joystick curve tuning and the overheat warning.

#include "vex.h"

namespace hh {
namespace driver {

// The defaults for every setting in driver.h. configure_robot() changes them.
DriveType drive_type = TANK;
DriveStyle style = SPLIT_ARCADE;
float stick_deadband = 5;
float throttle_curve = 0;
float turn_curve = 0;
float turn_scale = 1;
float strafe_scale = 1;
float active_brake_kp = 0;
float overheat_temp = 55;
bool curve_tuning = false;

// The SD card file the live-tuned curves are kept in. With driver profiles,
// each driver gets a file of their own, named after them.
static const char* curve_file() {
  static char file[40];
  if (profiles::count() == 0) return "hh_curves.txt";
  // Keep only letters and digits of the name, so it is a safe file name.
  char safe[13];
  int length = 0;
  for (const char* c = profiles::selected_name(); *c != 0 && length < 12; c++) {
    bool letter_or_digit = (*c >= 'a' && *c <= 'z') || (*c >= 'A' && *c <= 'Z') || (*c >= '0' && *c <= '9');
    if (letter_or_digit) safe[length++] = *c;
  }
  safe[length] = 0;
  snprintf(file, sizeof(file), "hh_curves_%s.txt", safe);
  return file;
}

// field_offset   the heading that counts as "away from the driver" (field-centric)
// braking        true while the active brake is holding the robot still
// brake_targets  where each side or wheel was when the sticks were let go
static float field_offset = 0;
static bool braking = false;
static float brake_targets[4];

// The curve popularized by team 5225A: full stick still gives full speed, but
// the middle of the stick is flattened for finer control.
float curve(float input, float strength) {
  if (strength == 0) return input;
  float e = exp(-strength / 10.0);
  return input * (e + exp((fabs(input) - 100.0) / 10.0) * (1.0 - e));
}

// One stick reading: ignore tiny movements, then apply the curve.
static float stick(float raw, float strength) {
  return curve(::deadband(raw, stick_deadband), strength);
}

void reset_field_centric() { field_offset = chassis.get_absolute_heading(); }

// Spins each drive unit at a percent, or holds position when all are zero.
// A unit is one side (tank) or one wheel (mecanum / X).
static void output(float* percents, int count) {
  bool moving = false;
  float biggest = 100;
  for (int i = 0; i < count; i++) {
    if (percents[i] != 0) moving = true;
    if (fabs(percents[i]) > biggest) biggest = fabs(percents[i]);
  }

  float positions[4];
  if (count == 2) {
    positions[0] = chassis.DriveL.position(vex::deg);
    positions[1] = chassis.DriveR.position(vex::deg);
  } else {
    positions[0] = chassis.DriveLF.position(vex::deg);
    positions[1] = chassis.DriveRF.position(vex::deg);
    positions[2] = chassis.DriveLB.position(vex::deg);
    positions[3] = chassis.DriveRB.position(vex::deg);
  }

  float volts[4];
  if (moving || active_brake_kp == 0) {
    braking = false;
    // Scale everything down together so a maxed-out wheel doesn't change the
    // direction the robot goes.
    for (int i = 0; i < count; i++) volts[i] = percents[i] / biggest * 12.0;
  } else {
    if (!braking) {
      braking = true;
      for (int i = 0; i < count; i++) brake_targets[i] = positions[i];
    }
    for (int i = 0; i < count; i++) {
      volts[i] = clamp(active_brake_kp * (brake_targets[i] - positions[i]), -12, 12);
    }
  }

  if (count == 2) {
    chassis.spin_side(true, volts[0]);
    chassis.spin_side(false, volts[1]);
  } else {
    chassis.DriveLF.spin(vex::fwd, volts[0], vex::volt);
    chassis.DriveRF.spin(vex::fwd, volts[1], vex::volt);
    chassis.DriveLB.spin(vex::fwd, volts[2], vex::volt);
    chassis.DriveRB.spin(vex::fwd, volts[3], vex::volt);
  }
}

// Reads the joysticks and drives. Sticks are numbered the VEX way:
// Axis3 = left stick up/down, Axis4 = left stick left/right,
// Axis2 = right stick up/down, Axis1 = right stick left/right.
void drive() {
  float left_y = Controller.Axis3.position();
  float left_x = Controller.Axis4.position();
  float right_y = Controller.Axis2.position();
  float right_x = Controller.Axis1.position();

  float forward = 0, turn = 0, strafe = 0;
  bool holonomic = drive_type != TANK;

  // Step 1: what does the driver want, for the stick layout in use?
  switch (style) {
    case TANK_STICKS: {
      float left = stick(left_y, throttle_curve);
      float right = stick(right_y, throttle_curve);
      forward = (left + right) / 2.0;
      turn = (left - right) / 2.0;
      break;
    }
    case SPLIT_ARCADE:
      forward = stick(left_y, throttle_curve);
      turn = stick(right_x, turn_curve) * turn_scale;
      break;
    case LEFT_ARCADE:
      forward = stick(left_y, throttle_curve);
      turn = stick(left_x, turn_curve) * turn_scale;
      break;
    case RIGHT_ARCADE:
      forward = stick(right_y, throttle_curve);
      turn = stick(right_x, turn_curve) * turn_scale;
      break;
    case FLIPPED_ARCADE:
      forward = stick(right_y, throttle_curve);
      turn = stick(left_x, turn_curve) * turn_scale;
      break;
    case HOLONOMIC:
    case FIELD_CENTRIC:
      forward = stick(left_y, throttle_curve);
      turn = stick(right_x, turn_curve) * turn_scale;
      if (holonomic) strafe = stick(left_x, throttle_curve) * strafe_scale;
      break;
  }

  // Step 2 (field-centric only): the stick says which way to go on the
  // field. Rotate that by the robot's heading to get which way the robot
  // itself has to move.
  if (style == FIELD_CENTRIC && holonomic) {
    float heading = to_rad(chassis.get_absolute_heading() - field_offset);
    float field_forward = forward;
    float field_strafe = strafe;
    forward = field_strafe * sin(heading) + field_forward * cos(heading);
    strafe = field_strafe * cos(heading) - field_forward * sin(heading);
  }

  // Step 3: mix forward / turn / strafe into a speed for each wheel
  // (mecanum, X-drive) or each side (tank).
  if (holonomic) {
    float wheels[4] = {
      forward + turn + strafe,   // left front
      forward - turn - strafe,   // right front
      forward + turn - strafe,   // left back
      forward - turn + strafe    // right back
    };
    output(wheels, 4);
  } else {
    float sides[2] = {forward + turn, forward - turn};
    output(sides, 2);
  }
}

// Writes the two curve numbers to the SD card as text, e.g. "3.0 2.5".
static void save_curves() {
  if (!Brain.SDcard.isInserted()) return;
  char text[40];
  int length = snprintf(text, sizeof(text), "%.1f %.1f", throttle_curve, turn_curve);
  Brain.SDcard.savefile(curve_file(), (uint8_t*)text, length);
}

// Reads them back at start-up. Only when curve tuning is on, so a team
// that sets its curves in code always gets exactly those.
void load_saved_curves() {
  if (!curve_tuning || !Brain.SDcard.isInserted() || !Brain.SDcard.exists(curve_file())) return;
  char text[40] = {0};
  Brain.SDcard.loadfile(curve_file(), (uint8_t*)text, sizeof(text) - 1);
  float throttle = 0, turning = 0;
  if (sscanf(text, "%f %f", &throttle, &turning) == 2) {
    throttle_curve = clamp(throttle, 0, 10);
    turn_curve = clamp(turning, 0, 10);
  }
}

// Live curve tuning: Left/Right change the throttle curve, Y/A the turn
// curve. `last` remembers each button from the previous loop so one
// press changes the value once.
static void tune_curves() {
  static bool last[4] = {false, false, false, false};
  static bool shown = false;
  bool now[4] = {Controller.ButtonLeft.pressing(), Controller.ButtonRight.pressing(),
                 Controller.ButtonY.pressing(), Controller.ButtonA.pressing()};
  bool changed = false;
  if (now[0] && !last[0]) { throttle_curve -= 0.5; changed = true; }
  if (now[1] && !last[1]) { throttle_curve += 0.5; changed = true; }
  if (now[2] && !last[2]) { turn_curve -= 0.5; changed = true; }
  if (now[3] && !last[3]) { turn_curve += 0.5; changed = true; }
  for (int i = 0; i < 4; i++) last[i] = now[i];

  if (changed) {
    throttle_curve = clamp(throttle_curve, 0, 10);
    turn_curve = clamp(turn_curve, 0, 10);
    save_curves();
  }
  if (changed || !shown) {
    shown = true;
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Curve T%.1f R%.1f   ", throttle_curve, turn_curve);
  }
}

// Called every 20 ms from usercontrol(). The first block spots the
// start of driver control (the first call, or the first after a gap
// while the robot was disabled or in autonomous) and tidies up.
void update() {
  static uint32_t last_update = 0;
  uint32_t now = now_ms();
  // A gap means driver control has just (re)started.
  if (last_update == 0 || now - last_update > 250) {
    braking = false;
    // A background motion left over from autonomous must not fight the driver.
    chassis.cancel_motion();
    // Mirroring is for autons only. One cut short by the end of autonomous
    // never got to turn it back off.
    if (!tuner::enabled) chassis.set_mirrored(false);
    tuner::reset();
    for (int i = 0; i < subsystem_count(); i++) {
      subsystem_at(i)->on_driver_start();
    }
  }
  last_update = now;

  // The PID tuner takes over the controller while it is turned on.
  if (tuner::enabled) {
    if (!tuner::update()) drive();
    return;
  }

  if (curve_tuning) tune_curves();
  update_buttons();
  // A one-button sequence that moves the chassis has the drive until it
  // finishes or the driver moves a stick.
  if (!Sequence::chassis_busy()) drive();
}

// Called by the background task. Once a second it finds the hottest
// motor on the robot and, if that is over overheat_temp, buzzes the
// controller and names it. At most one warning every 15 seconds.
void check_overheat() {
  static uint32_t last_check = 0;
  static uint32_t last_warning = 0;
  uint32_t now = now_ms();
  if (overheat_temp <= 0 || now - last_check < 1000) return;
  last_check = now;

  const char* hottest_name = "Drive";
  float hottest = chassis.DriveL.temperature(vex::temperatureUnits::celsius);
  float right = chassis.DriveR.temperature(vex::temperatureUnits::celsius);
  if (right > hottest) hottest = right;
  for (int i = 0; i < subsystem_count(); i++) {
    float temperature = subsystem_at(i)->temperature();
    if (temperature > hottest) {
      hottest = temperature;
      hottest_name = subsystem_at(i)->name;
    }
  }

  // A motor group only reports its first motor, and the drive has several.
  // Check every smart port too, so no hot motor is missed.
  static char port_name[12];
  for (int port = 1; port <= 21; port++) {
    float temperature = ports::temperature(port);
    if (temperature > hottest) {
      hottest = temperature;
      snprintf(port_name, sizeof(port_name), "Port %d", port);
      hottest_name = port_name;
    }
  }

  if (hottest >= overheat_temp && (last_warning == 0 || now - last_warning > 15000)) {
    last_warning = now;
    Controller.rumble("- -");
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("HOT: %s %dC      ", hottest_name, (int)hottest);
  }
}

}  // namespace driver
}  // namespace hh

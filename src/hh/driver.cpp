#include "vex.h"

namespace hh {
namespace driver {

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

static const char* CURVE_FILE = "hh_curves.txt";

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
    chassis.DriveL.spin(vex::fwd, volts[0], vex::volt);
    chassis.DriveR.spin(vex::fwd, volts[1], vex::volt);
  } else {
    chassis.DriveLF.spin(vex::fwd, volts[0], vex::volt);
    chassis.DriveRF.spin(vex::fwd, volts[1], vex::volt);
    chassis.DriveLB.spin(vex::fwd, volts[2], vex::volt);
    chassis.DriveRB.spin(vex::fwd, volts[3], vex::volt);
  }
}

void drive() {
  float left_y = Controller.Axis3.position();
  float left_x = Controller.Axis4.position();
  float right_y = Controller.Axis2.position();
  float right_x = Controller.Axis1.position();

  float forward = 0, turn = 0, strafe = 0;
  bool holonomic = drive_type != TANK;

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

  if (style == FIELD_CENTRIC && holonomic) {
    float heading = to_rad(chassis.get_absolute_heading() - field_offset);
    float field_forward = forward;
    float field_strafe = strafe;
    forward = field_strafe * sin(heading) + field_forward * cos(heading);
    strafe = field_strafe * cos(heading) - field_forward * sin(heading);
  }

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

static void save_curves() {
  if (!Brain.SDcard.isInserted()) return;
  char text[40];
  int length = snprintf(text, sizeof(text), "%.1f %.1f", throttle_curve, turn_curve);
  Brain.SDcard.savefile(CURVE_FILE, (uint8_t*)text, length);
}

void load_saved_curves() {
  if (!curve_tuning || !Brain.SDcard.isInserted() || !Brain.SDcard.exists(CURVE_FILE)) return;
  char text[40] = {0};
  Brain.SDcard.loadfile(CURVE_FILE, (uint8_t*)text, sizeof(text) - 1);
  float throttle = 0, turning = 0;
  if (sscanf(text, "%f %f", &throttle, &turning) == 2) {
    throttle_curve = clamp(throttle, 0, 10);
    turn_curve = clamp(turning, 0, 10);
  }
}

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

void update() {
  static uint32_t last_update = 0;
  uint32_t now = now_ms();
  // A gap means driver control has just (re)started.
  if (last_update == 0 || now - last_update > 250) {
    braking = false;
    for (int i = 0; i < subsystem_count(); i++) {
      subsystem_at(i)->on_driver_start();
    }
  }
  last_update = now;

  if (curve_tuning) tune_curves();
  update_buttons();
  drive();
}

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

  if (hottest >= overheat_temp && (last_warning == 0 || now - last_warning > 15000)) {
    last_warning = now;
    Controller.rumble("- -");
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("HOT: %s %dC      ", hottest_name, (int)hottest);
  }
}

}  // namespace driver
}  // namespace hh

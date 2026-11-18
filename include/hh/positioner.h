#pragma once
#include "hh/subsystem.h"
#include "hh/buttons.h"

namespace hh {

/**
 * Anything that moves to a position and stays there: lifts, arms, wall-stake
 * mechanisms, wrists.
 *
 *   hh::Positioner lift(LiftMotor, 12.0 / 60.0);
 *
 *   // in configure_robot():
 *   lift.add_preset("down", 0);
 *   lift.add_preset("high", 110);
 *   lift.bind_preset(Controller.ButtonUp, "high");
 *   lift.bind_preset(Controller.ButtonDown, "down");
 *
 *   // in an auton:
 *   lift.go_to("high");     // starts moving and returns right away
 *   lift.wait();            // optional: pause until it gets there
 *
 * Positions are in degrees of the mechanism (not the motor). Zero is wherever
 * the mechanism sits when the program starts. Positive voltage must make the
 * position go up; if it doesn't, flip `reversed` on the motor.
 */
class Positioner : public Subsystem {
public:
  /**
   * @param motors      The motors that move the mechanism.
   * @param gear_ratio  Driving gear teeth / driven gear teeth. A 12-tooth gear
   *                    on the motor turning a 60-tooth gear on the arm is
   *                    12.0 / 60.0. Use 1 for direct drive.
   */
  Positioner(vex::motor& motor, float gear_ratio = 1.0)
      : Positioner(vex::motor_group(motor), gear_ratio, 0) {}
  Positioner(vex::motor& motor_a, vex::motor& motor_b, float gear_ratio = 1.0)
      : Positioner(vex::motor_group(motor_a, motor_b), gear_ratio, 0) {}
  /** A motor group you already have. The 0 is ignored; it keeps C++ happy. */
  Positioner(vex::motor_group motors, float gear_ratio, int unused);

  // ---- Settings (change these in configure_robot()) ----

  /** PID constants. Output is in volts, error is in degrees. */
  float kp = 0.4;
  float ki = 0;
  float kd = 0.5;
  /** The integral only builds up when the error is smaller than this (degrees). */
  float starti = 10;
  /** Constant voltage to hold against gravity (elevator lifts). */
  float kg = 0;
  /** Voltage to hold an arm level. Fades out as the arm points up or down. */
  float kg_arm = 0;
  /** The position (degrees) where the arm is level. Used with kg_arm. */
  float arm_level_position = 0;
  /** Top voltage for position moves, out of 12. */
  float max_voltage = 12;
  /** "There" means within settle_error degrees for settle_time ms. */
  float settle_error = 2;
  int settle_time = 100;
  /** The mechanism won't be driven past these positions (degrees). */
  bool use_limits = false;
  float min_position = 0;
  float max_position = 0;

  // ---- Commands ----

  /** Names a position so you can go_to() it later. Up to 12 presets. */
  Positioner& add_preset(const char* preset_name, float position);
  /** Starts moving to a preset. Returns right away. */
  void go_to(const char* preset_name);
  /** Starts moving to a position in degrees. Returns right away. */
  void go_to(float position);
  /** Waits until the mechanism arrives or `timeout_ms` passes. */
  void wait(int timeout_ms = 3000);
  /** Holds the mechanism where it is right now. */
  void hold();
  /** Drives with a voltage (out of 12) until told otherwise. For driver control. */
  void move(float voltage);
  /** Lets the motors go limp. */
  void stop();
  /** Calls the current position `position` degrees (default 0). */
  void zero(float position = 0);
  /** Sets both soft limits and turns them on. */
  void set_limits(float min_position, float max_position);
  /** Measures position with a rotation sensor instead of the motor encoders. */
  void use_rotation_sensor(vex::rotation& sensor, float sensor_ratio = 1.0);

  float position();
  float target() const { return target_; }
  bool is_settled() const;

  /** Pressing the button sends the mechanism to a preset. */
  void bind_preset(Button button, const char* preset_name);
  /** Hold `up` or `down` to move by hand. It holds position on release. */
  void bind_manual(Button up, Button down, float voltage = 12);

  void update() override;
  /** Forgets the target, so it doesn't jump back to it when the motors return. */
  void on_pause() override;
  float temperature() override;

  /** The motors, for anything this class doesn't cover. */
  vex::motor_group motors;

private:
  enum Mode { IDLE, POSITION, MANUAL };
  struct Preset {
    const char* name;
    float position;
  };

  float feedforward(float position);

  static const int MAX_PRESETS = 12;
  Preset presets_[MAX_PRESETS];
  int preset_count_ = 0;

  float gear_ratio_;
  vex::rotation* sensor_ = nullptr;
  float sensor_ratio_ = 1;
  float offset_ = 0;

  Mode mode_ = IDLE;
  float target_ = 0;
  float manual_voltage_ = 0;
  float integral_ = 0;
  float previous_error_ = 0;
  int settled_ms_ = 0;
};

}  // namespace hh

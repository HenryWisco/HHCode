#pragma once
#include "hh/subsystem.h"
#include "hh/buttons.h"

namespace hh {

/**
 * Anything that has to hold an exact speed: flywheels, launcher rollers.
 *
 *   hh::Velocity flywheel(FlywheelMotor, 3600);
 *   flywheel.set_rpm(2400);
 *   if (flywheel.ready()) { ...shoot... }
 */
class Velocity : public Subsystem {
public:
  /**
   * @param motors   The motors that spin the mechanism.
   * @param max_rpm  The mechanism's top speed: the motor cartridge speed times
   *                 your gearing. A 600 rpm motor geared 6:1 for speed is 3600.
   */
  Velocity(vex::motor& motor, float max_rpm) : Velocity(vex::motor_group(motor), max_rpm, 0) {}
  Velocity(vex::motor& motor_a, vex::motor& motor_b, float max_rpm)
      : Velocity(vex::motor_group(motor_a, motor_b), max_rpm, 0) {}
  /** A motor group you already have. The 0 is ignored; it keeps C++ happy. */
  Velocity(vex::motor_group motors, float max_rpm, int unused);

  // ---- Settings (change these in configure_robot()) ----

  /** Feedforward. 1.0 sends the voltage that should give the target speed. */
  float kf = 1.0;
  /** Extra volts per 1% of top speed that the mechanism is off by. */
  float kp = 0.3;
  /** Slowly adds voltage while it stays off target. Volts per 1% per second. */
  float ki = 0;
  /** The most the voltage may change per second. 0 means no limit. */
  float ramp = 0;
  /** ready() is true when within this many rpm of the target... */
  float ready_tolerance;
  /** ...for this long (ms). */
  int ready_time = 100;

  // ---- Commands ----

  void set_rpm(float rpm);
  void stop();

  float rpm();
  float target_rpm() const { return target_rpm_; }
  bool is_spinning() const { return target_rpm_ != 0; }
  /** True once the speed has settled on the target. */
  bool ready() const;
  /** Waits until ready() or `timeout_ms` passes. */
  void wait_until_ready(int timeout_ms = 3000);

  /** Each press turns the mechanism on at `rpm` or off. */
  void bind_toggle(Button button, float rpm);

  void update() override;
  float temperature() override;

  /** The motors, for anything this class doesn't cover. */
  vex::motor_group motors;

private:
  float max_rpm_;
  float target_rpm_ = 0;
  float integral_ = 0;
  float output_ = 0;
  int ready_ms_ = 0;
};

}  // namespace hh

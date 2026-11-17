#pragma once
#include "hh/subsystem.h"
#include "hh/buttons.h"

namespace hh {

/**
 * Anything that just spins: intakes, rollers, conveyors.
 *
 *   hh::Spinner intake(IntakeMotor);              // or (IntakeLeft, IntakeRight)
 *   intake.bind_hold(Controller.ButtonR1, Controller.ButtonR2);  // in configure_robot()
 *   intake.spin();                                               // in an auton
 *
 * Positive must be the "in" direction. If it runs backwards, flip `reversed`
 * on the motor in robot-config.cpp.
 */
class Spinner : public Subsystem {
public:
  /** One or more motors: hh::Spinner intake(IntakeLeft, IntakeRight); */
  template <typename... More>
  Spinner(vex::motor& first, More&... more) : Spinner(vex::motor_group(first, more...), 0) {}
  /** A motor group you already have. The 0 is ignored; it keeps C++ happy. */
  Spinner(vex::motor_group motors, int unused);

  // ---- Settings (change these in configure_robot()) ----

  /** Speed used by spin() and reverse(), in percent (0 to 100). */
  float speed = 100;
  /** false: send a voltage (strongest). true: hold an exact speed under load. */
  bool velocity_mode = false;
  /** What the motors do when stopped: vex::coast, vex::brake or vex::hold. */
  vex::brakeType stop_mode = vex::brakeType::coast;

  /** Turn on to back out by itself when something gets stuck. */
  bool jam_protection = false;
  /** Counts as stuck when slower than this (percent of top speed)... */
  float jam_speed = 5;
  /** ...for this long (ms). */
  int jam_time = 200;
  /** How long to run backwards to clear a jam (ms). */
  int unjam_time = 250;
  /** How long after starting before jams are checked (ms). Covers spin-up. */
  int jam_grace_time = 300;

  // ---- Commands ----

  void spin();
  void spin(float percent);
  void reverse();
  void reverse(float percent);
  void stop();

  /** The speed last asked for, in percent. Negative is reverse, 0 is stopped. */
  float commanded() const { return command_; }
  bool is_spinning() const { return command_ != 0; }
  /** True while it is backing out of a jam. */
  bool is_unjamming() const { return unjam_end_ != 0; }
  /** How many jams have been cleared since the program started. */
  int jam_count() const { return jam_count_; }

  /** Limits motor current (amps, up to 2.5). Lower it to be gentle on game pieces. */
  void set_current_limit(float amps);

  /** Spins forward while `forward` is held and backward while `backward` is held. */
  void bind_hold(Button forward, Button backward);
  /** Spins forward while the button is held. */
  void bind_hold(Button forward);
  /** Each press turns it on or off. The two buttons pick the direction. */
  void bind_toggle(Button forward, Button backward);
  /** Each press turns it on or off. */
  void bind_toggle(Button forward);

  void update() override;
  void on_driver_start() override;
  void on_disable() override;
  float temperature() override;

  /** The motors, for anything this class doesn't cover. */
  vex::motor_group motors;

private:
  void apply(float percent);

  float command_ = 0;
  uint32_t command_start_ = 0;
  uint32_t slow_since_ = 0;
  uint32_t unjam_end_ = 0;
  int jam_count_ = 0;
};

}  // namespace hh

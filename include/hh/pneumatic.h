#pragma once
#include "hh/subsystem.h"
#include "hh/buttons.h"

namespace hh {

/**
 * A piston (or two pistons that always move together): clamps, wings,
 * doinkers, hangs.
 *
 *   hh::Pneumatic mogo_clamp(Brain.ThreeWirePort.A);
 *   mogo_clamp.bind_toggle(Controller.ButtonA);   // in configure_robot()
 *   mogo_clamp.extend();                          // in an auton
 */
class Pneumatic : public Subsystem {
public:
  /** One solenoid. `start_extended` is the state when the program starts. */
  Pneumatic(vex::triport::port& port, bool start_extended = false);
  /** Two solenoids that fire together. */
  Pneumatic(vex::triport::port& port_a, vex::triport::port& port_b, bool start_extended = false);

  /** Set true if "extend" and "retract" come out backwards on your robot. */
  bool reversed = false;

  void extend();
  void retract();
  void toggle();
  void set(bool extended);
  bool is_extended() const { return extended_; }

  /** Extends now and retracts by itself after `ms` milliseconds. Doesn't block. */
  void pulse(int ms);

  /** Each press of the button flips the piston. */
  void bind_toggle(Button button);
  /** Extended while the button is held, retracted when released. */
  void bind_hold(Button button);

  void init() override;
  void update() override;

private:
  void apply();

  // a_          the solenoid (always there)
  // b_storage_  memory for a second solenoid; b_ points into it when used
  // extended_   the state last asked for
  // pulse_end_  when a pulse() should retract (0 = no pulse running)
  vex::digital_out a_;
  alignas(vex::digital_out) char b_storage_[sizeof(vex::digital_out)];
  vex::digital_out* b_ = nullptr;
  bool extended_;
  uint32_t pulse_end_ = 0;
};

}  // namespace hh

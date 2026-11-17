#pragma once
#include "vex.h"

namespace hh {

/**
 * Pneumatic: anything driven by a piston. Claws, clamps, wings.
 *
 * Set it up in robot-config.cpp:
 *   digital_out ClawPiston = digital_out(Brain.ThreeWirePort.A);
 *   hh::Pneumatic claw = hh::Pneumatic(ClawPiston);
 *
 * Then use it anywhere:
 *   claw.extend();
 *   claw.retract();
 *   claw.toggle();          // flips to the other position
 *
 * If two solenoids should always move together, give both:
 *   hh::Pneumatic wings = hh::Pneumatic(LeftWing, RightWing);
 */
class Pneumatic
{
public:
  Pneumatic(digital_out &solenoid);
  Pneumatic(digital_out &solenoid, digital_out &second_solenoid);

  void extend();
  void retract();
  void toggle();

  /** Sets the position directly. true is extended, false is retracted. */
  void set(bool extended);

  bool is_extended();

  /**
   * Call this once if your piston is plumbed backward, so that
   * extend() actually extends it.
   */
  void set_reversed(bool reversed);

private:
  digital_out *solenoid;
  digital_out *second_solenoid = nullptr;
  bool extended = false;
  bool reversed = false;
};

}

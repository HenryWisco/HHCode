#pragma once
#include "vex.h"

namespace hh {

/**
 * Spinner: anything that just spins. Roller intakes, conveyors, rollers.
 *
 * Set it up in robot-config.cpp:
 *   hh::Spinner intake = hh::Spinner(motor_group(IntakeMotor));
 *
 * Then use it anywhere (autons or driver control):
 *   intake.spin();          // full speed forward
 *   intake.spin(8);         // forward at 8 volts (max is 12)
 *   intake.reverse();       // full speed backward
 *   intake.stop();
 *   intake.spin_for(500);   // spin for half a second, then stop
 */
class Spinner
{
public:
  motor_group motors;

  /** Voltage used by spin() and reverse() when you don't give a number. Max is 12. */
  float default_voltage = 12;

  Spinner(motor_group motors);

  void spin();
  void spin(float voltage);
  void reverse();
  void reverse(float voltage);
  void stop();

  /** Spins for a set time in milliseconds, then stops. Waits until it's done. */
  void spin_for(float time_ms);
  void spin_for(float time_ms, float voltage);

  /** What the motors do when stopped: coast (default), brake, or hold. */
  void set_stop_mode(brakeType mode);

  /** True while the Spinner has been told to spin. */
  bool is_spinning();

private:
  bool spinning = false;
};

}

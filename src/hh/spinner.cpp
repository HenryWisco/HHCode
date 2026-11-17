#include "vex.h"

namespace hh {

/**
 * Spinner constructor.
 *
 * @param motors The motors that spin this mechanism, i.e. motor_group(IntakeMotor).
 */

Spinner::Spinner(motor_group motors) :
  motors(motors)
{
  this->motors.setStopping(coast);
}

/**
 * Spins forward at the default voltage.
 */

void Spinner::spin(){
  spin(default_voltage);
}

/**
 * Spins at the given voltage.
 *
 * @param voltage Voltage out of 12. Negative spins backward.
 */

void Spinner::spin(float voltage){
  motors.spin(fwd, clamp(voltage, -12, 12), volt);
  spinning = true;
}

/**
 * Spins backward at the default voltage.
 */

void Spinner::reverse(){
  spin(-default_voltage);
}

/**
 * Spins backward at the given voltage.
 *
 * @param voltage Voltage out of 12, as a positive number.
 */

void Spinner::reverse(float voltage){
  spin(-voltage);
}

void Spinner::stop(){
  motors.stop();
  spinning = false;
}

/**
 * Spins forward for a set time, then stops. This waits until it's
 * finished, so the next line of your auton runs after the spin.
 *
 * @param time_ms Time to spin in milliseconds.
 */

void Spinner::spin_for(float time_ms){
  spin_for(time_ms, default_voltage);
}

void Spinner::spin_for(float time_ms, float voltage){
  spin(voltage);
  wait(time_ms, msec);
  stop();
}

void Spinner::set_stop_mode(brakeType mode){
  motors.setStopping(mode);
}

bool Spinner::is_spinning(){
  return(spinning);
}

}

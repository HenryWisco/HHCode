#include "vex.h"

namespace hh {

/**
 * Positioner constructor for a mechanism driven straight off the motor.
 *
 * @param motors The motors that move this mechanism, i.e. motor_group(LiftMotor).
 */

Positioner::Positioner(motor_group motors) :
  Positioner(motors, 1)
{}

/**
 * Positioner constructor for a geared mechanism.
 *
 * @param motors The motors that move this mechanism.
 * @param gear_ratio How many times the motor turns for the mechanism to turn
 * once. A 12-tooth gear on the motor driving a 60-tooth gear on the arm is 5.
 */

Positioner::Positioner(motor_group motors, float gear_ratio) :
  motors(motors),
  gear_ratio(gear_ratio)
{
  this->motors.setStopping(vex::hold);
}

/**
 * Saves a named position so you can use go_to("name") later.
 *
 * @param name Any name you like, i.e. "high".
 * @param degrees Position in degrees of the mechanism.
 */

void Positioner::add_preset(const char *name, float degrees){
  if (preset_count >= max_presets){
    printf("hh::Positioner: too many presets, \"%s\" was not added\n", name);
    return;
  }
  preset_names[preset_count] = name;
  preset_degrees[preset_count] = degrees;
  preset_count++;
}

/**
 * Starts moving to a named position. Does not wait.
 * If the name was never added with add_preset(), nothing moves and a
 * message prints to the terminal.
 *
 * @param preset_name The name given to add_preset().
 */

void Positioner::go_to(const char *preset_name){
  for (int i = 0; i < preset_count; i++){
    if (strcmp(preset_names[i], preset_name) == 0){
      go_to(preset_degrees[i]);
      return;
    }
  }
  printf("hh::Positioner: no preset named \"%s\"\n", preset_name);
}

/**
 * Starts moving to a position. Does not wait.
 *
 * @param degrees Position in degrees of the mechanism.
 */

void Positioner::go_to(float degrees){
  target = degrees;
  moving_manually = false;
  motors.spinToPosition(degrees*gear_ratio, deg, speed_percent, velocityUnits::pct, false);
}

void Positioner::wait(){
  float time_spent = 0;
  while (!is_done() && time_spent < timeout){
    task::sleep(10);
    time_spent += 10;
  }
}

bool Positioner::is_done(){
  return(fabs(target - get_position()) < settle_error);
}

float Positioner::get_position(){
  return(motors.position(deg)/gear_ratio);
}

void Positioner::zero(){
  motors.resetPosition();
  target = 0;
}

/**
 * Moves at a voltage for driver control. When the voltage goes back
 * to 0, the mechanism holds wherever it is.
 *
 * @param voltage Voltage out of 12. Negative goes down.
 */

void Positioner::manual(float voltage){
  if (voltage == 0){
    if (moving_manually){
      hold();
    }
    return;
  }
  moving_manually = true;
  motors.spin(fwd, clamp(voltage, -12, 12), volt);
}

void Positioner::hold(){
  moving_manually = false;
  target = get_position();
  motors.stop(vex::hold);
}

}

#include "vex.h"

namespace hh {

/**
 * Pneumatic constructor for one solenoid.
 *
 * @param solenoid The digital_out you made in robot-config.cpp.
 */

Pneumatic::Pneumatic(digital_out &solenoid) :
  solenoid(&solenoid)
{}

/**
 * Pneumatic constructor for two solenoids that always move together.
 */

Pneumatic::Pneumatic(digital_out &solenoid, digital_out &second_solenoid) :
  solenoid(&solenoid),
  second_solenoid(&second_solenoid)
{}

void Pneumatic::extend(){
  set(true);
}

void Pneumatic::retract(){
  set(false);
}

void Pneumatic::toggle(){
  set(!extended);
}

/**
 * Sets the piston position.
 *
 * @param extended true to extend, false to retract.
 */

void Pneumatic::set(bool extended){
  this->extended = extended;
  bool output = reversed ? !extended : extended;
  solenoid->set(output);
  if (second_solenoid != nullptr){
    second_solenoid->set(output);
  }
}

bool Pneumatic::is_extended(){
  return(extended);
}

void Pneumatic::set_reversed(bool reversed){
  this->reversed = reversed;
}

}

// hh::Pto: drive motors that a piston can shift over to a mechanism.
//
// The chassis keeps two lists, chassis.pto_left and chassis.pto_right, of
// extra motors that drive with each side. Joining the drive means putting
// the PTO motors in those lists; leaving means taking them out.

#include "vex.h"

namespace hh {

Pto::Pto(vex::motor& left, vex::motor& right) : left_{&left}, right_{&right} { name = "PTO"; }

Pto::Pto(vex::motor& left_a, vex::motor& left_b, vex::motor& right_a, vex::motor& right_b)
    : left_{&left_a, &left_b}, right_{&right_a, &right_b} {
  name = "PTO";
}

void Pto::use_piston(Pneumatic& piston, bool extended_is_drive) {
  piston_ = &piston;
  extended_is_drive_ = extended_is_drive;
}

void Pto::controls(Subsystem& mechanism) { mechanism_ = &mechanism; }

// Does the shift, in an order that never has two things powering the motors.
void Pto::apply(bool driving) {
  driving_ = driving;

  // Hand the motors over first, so nothing is still powering them the old
  // way when the piston moves.
  if (mechanism_ != nullptr) {
    // Going to the drive: let the mechanism forget what it was doing first.
    if (driving && !mechanism_->paused) mechanism_->on_pause();
    mechanism_->paused = driving;
  }
  chassis.pto_left.clear();
  chassis.pto_right.clear();
  for (vex::motor* motor : left_) motor->stop(vex::brakeType::coast);
  for (vex::motor* motor : right_) motor->stop(vex::brakeType::coast);

  if (piston_ != nullptr) piston_->set(driving == extended_is_drive_);
  if (shift_time > 0) vex::task::sleep(shift_time);

  if (driving) {
    chassis.pto_left = left_;
    chassis.pto_right = right_;
  }
}

void Pto::to_drive() {
  if (!driving_) apply(true);
}

void Pto::to_mechanism() {
  if (driving_) apply(false);
}

void Pto::toggle() { apply(!driving_); }

void Pto::bind_toggle(Button button) {
  on_press(button, [this] { toggle(); });
}

// At start-up, put the PTO where the team said it starts.
void Pto::init() { apply(start_on_drive); }

// The hottest PTO motor, for the overheat warning.
float Pto::temperature() {
  float hottest = 0;
  for (vex::motor* motor : left_) hottest = fmax(hottest, motor->temperature(vex::temperatureUnits::celsius));
  for (vex::motor* motor : right_) hottest = fmax(hottest, motor->temperature(vex::temperatureUnits::celsius));
  return hottest;
}

}  // namespace hh

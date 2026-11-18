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

void Pto::apply(bool driving) {
  driving_ = driving;

  // Hand the motors over first, so nothing is still powering them the old
  // way when the piston moves.
  if (mechanism_ != nullptr) {
    // Going to the drive: let the mechanism forget what it was doing first.
    if (driving && !mechanism_->paused) mechanism_->on_disable();
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

void Pto::init() { apply(start_on_drive); }

float Pto::temperature() {
  float hottest = 0;
  for (vex::motor* motor : left_) hottest = fmax(hottest, motor->temperature(vex::temperatureUnits::celsius));
  for (vex::motor* motor : right_) hottest = fmax(hottest, motor->temperature(vex::temperatureUnits::celsius));
  return hottest;
}

}  // namespace hh

// hh::Velocity: holds a mechanism (a flywheel) at an exact speed.
//
// update() runs every 10 ms. The voltage it sends has two parts:
//   feedforward  the voltage that should give the target speed (kf)
//   correction   extra voltage for however far off the speed really is (kp, ki)

#include "vex.h"

namespace hh {

Velocity::Velocity(vex::motor_group motors, float max_rpm, int)
    : ready_tolerance(max_rpm * 0.03), motors(motors), max_rpm_(max_rpm) {
  name = "Flywheel";
}

// Sets the target speed. 0 means stop.
void Velocity::set_rpm(float rpm) {
  if (rpm == 0 || paused) {   // paused: the motors are doing something else (see hh::Pto)
    stop();
    return;
  }
  target_rpm_ = clamp(rpm, -max_rpm_, max_rpm_);
  ready_ms_ = 0;
}

// Stops and clears the controller memory, so the next spin-up starts fresh.
void Velocity::stop() {
  target_rpm_ = 0;
  integral_ = 0;
  output_ = 0;
  ready_ms_ = 0;
  if (!paused) motors.stop(vex::brakeType::coast);
}

// The speed right now, in rpm of the mechanism (not the motor).
float Velocity::rpm() { return motors.velocity(vex::percent) / 100.0 * max_rpm_; }

// "Ready" = within ready_tolerance of the target for ready_time.
bool Velocity::ready() const { return target_rpm_ != 0 && ready_ms_ >= ready_time; }

void Velocity::wait_until_ready(int timeout_ms) {
  uint32_t start = now_ms();
  while (!ready() && now_ms() - start < (uint32_t)timeout_ms) {
    vex::task::sleep(10);
  }
}

void Velocity::bind_toggle(Button button, float rpm) {
  on_press(button, [this, rpm] { if (is_spinning()) stop(); else set_rpm(rpm); });
}

void Velocity::update() {
  if (target_rpm_ == 0) return;

  // Work in percent of top speed so the constants suit any gearing.
  float target_percent = target_rpm_ / max_rpm_ * 100.0;
  float error = target_percent - motors.velocity(vex::percent);

  integral_ = clamp(integral_ + ki * error * 0.01, -4, 4);
  float wanted = kf * target_percent * 0.12 + kp * error + integral_;
  wanted = clamp(wanted, -12, 12);

  // Optional ramp: limit how fast the voltage may change, to be gentle on gears.
  if (ramp > 0) {
    float step = ramp * 0.01;
    wanted = clamp(wanted, output_ - step, output_ + step);
  }
  output_ = wanted;
  motors.spin(vex::fwd, output_, vex::volt);

  // Count how long the speed has been on target. ready() reads this.
  if (fabs(error) / 100.0 * max_rpm_ <= ready_tolerance) {
    ready_ms_ += 10;
  } else {
    ready_ms_ = 0;
  }
}

float Velocity::temperature() { return motors.temperature(vex::temperatureUnits::celsius); }

}  // namespace hh

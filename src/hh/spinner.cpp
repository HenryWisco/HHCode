#include "vex.h"

namespace hh {

Spinner::Spinner(vex::motor_group motors, int) : motors(motors) { name = "Spinner"; }

void Spinner::apply(float percent) {
  if (paused) return;   // the motors are doing something else (see hh::Pto)
  if (percent == 0) {
    motors.stop(stop_mode);
  } else if (velocity_mode) {
    motors.spin(vex::fwd, percent, vex::percent);
  } else {
    motors.spin(vex::fwd, percent * 0.12, vex::volt);
  }
}

void Spinner::spin(float percent) {
  if (paused) percent = 0;
  percent = clamp(percent, -100, 100);
  if (percent != command_) {
    command_start_ = now_ms();
    slow_since_ = 0;
  }
  command_ = percent;
  unjam_end_ = 0;
  // Stopping always wins. Anything else waits for an override to finish.
  if (percent == 0) override_end_ = 0;
  if (override_end_ == 0) apply(command_);
}

void Spinner::override_for(float percent, int ms) {
  if (paused) return;
  unjam_end_ = 0;
  override_end_ = now_ms() + ms;
  apply(clamp(percent, -100, 100));
}

void Spinner::spin() { spin(speed); }

void Spinner::reverse(float percent) { spin(-percent); }

void Spinner::reverse() { spin(-speed); }

void Spinner::stop() { spin(0); }

void Spinner::set_current_limit(float amps) { motors.setMaxTorque(amps, vex::currentUnits::amp); }

void Spinner::bind_hold(Button forward, Button backward) {
  while_held(forward, [this] { spin(); }, [this] { if (command_ > 0) stop(); });
  while_held(backward, [this] { reverse(); }, [this] { if (command_ < 0) stop(); });
}

void Spinner::bind_hold(Button forward) {
  while_held(forward, [this] { spin(); }, [this] { stop(); });
}

void Spinner::bind_toggle(Button forward, Button backward) {
  on_press(forward, [this] { if (command_ > 0) stop(); else spin(); });
  on_press(backward, [this] { if (command_ < 0) stop(); else reverse(); });
}

void Spinner::bind_toggle(Button forward) {
  on_press(forward, [this] { if (command_ != 0) stop(); else spin(); });
}

void Spinner::update() {
  uint32_t now = now_ms();

  // Finish an override, then go back to what was asked for.
  if (override_end_ != 0) {
    if (now >= override_end_) {
      override_end_ = 0;
      command_start_ = now;
      slow_since_ = 0;
      apply(command_);
    }
    return;
  }
  if (!jam_protection) return;

  // Finish backing out, then go back to what was asked for.
  if (unjam_end_ != 0) {
    if (now >= unjam_end_) {
      unjam_end_ = 0;
      command_start_ = now;
      slow_since_ = 0;
      apply(command_);
    }
    return;
  }

  if (command_ == 0 || now - command_start_ < (uint32_t)jam_grace_time) {
    slow_since_ = 0;
    return;
  }

  if (fabs(motors.velocity(vex::percent)) < jam_speed) {
    if (slow_since_ == 0) slow_since_ = now;
    if (now - slow_since_ >= (uint32_t)jam_time) {
      jam_count_++;
      unjam_end_ = now + unjam_time;
      apply(command_ > 0 ? -100 : 100);
    }
  } else {
    slow_since_ = 0;
  }
}

void Spinner::on_driver_start() { stop(); }

void Spinner::on_disable() { stop(); }

float Spinner::temperature() { return motors.temperature(vex::temperatureUnits::celsius); }

}  // namespace hh

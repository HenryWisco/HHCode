#include "vex.h"

namespace hh {

Positioner::Positioner(vex::motor_group motors, float gear_ratio, int)
    : motors(motors), gear_ratio_(gear_ratio) {
  name = "Positioner";
}

Positioner& Positioner::add_preset(const char* preset_name, float position) {
  if (preset_count_ < MAX_PRESETS) {
    presets_[preset_count_++] = {preset_name, position};
  }
  return *this;
}

void Positioner::go_to(const char* preset_name) {
  for (int i = 0; i < preset_count_; i++) {
    if (strcmp(presets_[i].name, preset_name) == 0) {
      go_to(presets_[i].position);
      return;
    }
  }
  printf("%s: no preset called \"%s\"\n", name, preset_name);
}

void Positioner::go_to(float position) {
  if (paused) return;   // the motors are doing something else (see hh::Pto)
  if (use_limits) position = clamp(position, min_position, max_position);
  target_ = position;
  integral_ = 0;
  previous_error_ = target_ - this->position();
  settled_ms_ = 0;
  mode_ = POSITION;
}

void Positioner::wait(int timeout_ms) {
  uint32_t start = now_ms();
  while (!is_settled() && now_ms() - start < (uint32_t)timeout_ms) {
    vex::task::sleep(10);
  }
}

void Positioner::hold() { go_to(position()); }

void Positioner::move(float voltage) {
  if (paused) return;
  manual_voltage_ = clamp(voltage, -12, 12);
  mode_ = MANUAL;
}

void Positioner::stop() {
  mode_ = IDLE;
  if (!paused) motors.stop(vex::brakeType::coast);
}

void Positioner::zero(float position) {
  offset_ = 0;
  offset_ = position - this->position();
  // The old target is now in the wrong place, so stay where we are instead.
  if (mode_ == POSITION) hold();
}

void Positioner::set_limits(float min_position, float max_position) {
  this->min_position = min_position;
  this->max_position = max_position;
  use_limits = true;
}

void Positioner::use_rotation_sensor(vex::rotation& sensor, float sensor_ratio) {
  sensor_ = &sensor;
  sensor_ratio_ = sensor_ratio;
}

float Positioner::position() {
  float raw = sensor_ != nullptr ? sensor_->position(vex::deg) * sensor_ratio_
                                 : motors.position(vex::deg) * gear_ratio_;
  return raw + offset_;
}

bool Positioner::is_settled() const { return mode_ == POSITION && settled_ms_ >= settle_time; }

void Positioner::bind_preset(Button button, const char* preset_name) {
  on_press(button, [this, preset_name] { go_to(preset_name); });
}

void Positioner::bind_manual(Button up, Button down, float voltage) {
  while_held(up, [this, voltage] { move(voltage); },
             [this] { if (mode_ == MANUAL && manual_voltage_ > 0) hold(); });
  while_held(down, [this, voltage] { move(-voltage); },
             [this] { if (mode_ == MANUAL && manual_voltage_ < 0) hold(); });
}

float Positioner::feedforward(float position) {
  return kg + kg_arm * cos(to_rad(position - arm_level_position));
}

void Positioner::update() {
  if (mode_ == IDLE) return;
  float current = position();

  if (mode_ == MANUAL) {
    bool past_top = use_limits && manual_voltage_ > 0 && current >= max_position;
    bool past_bottom = use_limits && manual_voltage_ < 0 && current <= min_position;
    if (past_top || past_bottom) {
      hold();
    } else {
      motors.spin(vex::fwd, manual_voltage_, vex::volt);
    }
    return;
  }

  float error = target_ - current;
  if (fabs(error) < starti) integral_ += error;
  if ((error > 0) != (previous_error_ > 0)) integral_ = 0;
  float output = kp * error + ki * integral_ + kd * (error - previous_error_);
  previous_error_ = error;

  output = clamp(output, -max_voltage, max_voltage) + feedforward(current);
  motors.spin(vex::fwd, clamp(output, -12, 12), vex::volt);

  if (fabs(error) < settle_error) {
    settled_ms_ += 10;
  } else {
    settled_ms_ = 0;
  }
}

float Positioner::temperature() { return motors.temperature(vex::temperatureUnits::celsius); }

}  // namespace hh

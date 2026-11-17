#include "vex.h"

namespace hh {

ColorSorter::ColorSorter(vex::optical& sensor) : sensor_(&sensor) { name = "Sorter"; }

void ColorSorter::keep(Color color) {
  keep_ = color;
  flagged_ = false;
  wrong_ = false;
  pending_ = false;
}

void ColorSorter::set_active(bool active) {
  active_ = active;
  if (!active) pending_ = false;
}

bool ColorSorter::in_range(float hue, float min, float max) {
  if (min <= max) return hue >= min && hue <= max;
  return hue >= min || hue <= max;   // the range wraps around 0
}

Color ColorSorter::seen() {
  if (use_proximity && !sensor_->isNearObject()) return NO_COLOR;
  float hue = sensor_->hue();
  if (in_range(hue, red_hue_min, red_hue_max)) return RED;
  if (in_range(hue, blue_hue_min, blue_hue_max)) return BLUE;
  return NO_COLOR;
}

void ColorSorter::eject_with(Pneumatic& piston) {
  Pneumatic* p = &piston;
  start_ = [p] { p->extend(); };
  finish_ = [p] { p->retract(); };
}

void ColorSorter::eject_with(Spinner& spinner, float percent) {
  Spinner* s = &spinner;
  start_ = [this, s, percent] { s->override_for(percent, eject_time); };
  finish_ = nullptr;
}

void ColorSorter::eject_with(Action start, Action finish) {
  start_ = start;
  finish_ = finish;
}

void ColorSorter::bind_toggle(Button button) {
  on_press(button, [this] { set_active(!active_); });
}

void ColorSorter::init() {
  if (light_power > 0) {
    sensor_->setLightPower(light_power, vex::percent);
    sensor_->setLight(vex::ledState::on);
  }
}

void ColorSorter::end_eject() {
  if (!ejecting_) return;
  ejecting_ = false;
  if (finish_) finish_();
}

void ColorSorter::on_disable() {
  pending_ = false;
  end_eject();
}

void ColorSorter::update() {
  uint32_t now = now_ms();

  // An eject that has started always finishes, even if sorting is turned off.
  if (ejecting_ && now >= eject_end_) end_eject();

  if (is_active()) {
    Color color = seen();
    bool wrong = color != NO_COLOR && color != keep_;
    if (wrong && !wrong_) wrong_since_ = now;
    wrong_ = wrong;
    if (!wrong) {
      flagged_ = false;   // the piece has gone past; ready for the next one
    } else if (!flagged_ && now - wrong_since_ >= (uint32_t)confirm_time) {
      flagged_ = true;    // one eject per piece
      pending_ = true;
      pending_at_ = now + eject_delay;
    }
  } else {
    wrong_ = false;
    flagged_ = false;
  }

  if (pending_ && now >= pending_at_) {
    pending_ = false;
    ejecting_ = true;
    eject_end_ = now + eject_time;
    rejected_++;
    if (start_) start_();
  }
}

}  // namespace hh

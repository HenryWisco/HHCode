// hh::ColorSorter: throws out game pieces of the wrong color.
//
// update() runs every 10 ms. A piece goes through these steps:
//   1. wrong_     the sensor sees the wrong color
//   2. flagged_   it has been seen for confirm_time, so it is real
//   3. pending_   waiting eject_delay for it to reach the ejector
//   4. ejecting_  the piston is out (or the intake is reversed) for eject_time

#include "vex.h"

namespace hh {

ColorSorter::ColorSorter(vex::optical& sensor) : sensor_(&sensor) { name = "Sorter"; }

// Choosing a color also clears any piece that was half way through the steps.
void ColorSorter::keep(Color color) {
  keep_ = color;
  flagged_ = false;
  wrong_ = false;
  pending_ = false;
}

// Pausing cancels an eject that was waiting, but not one that has started.
void ColorSorter::set_active(bool active) {
  active_ = active;
  if (!active) pending_ = false;
}

// Is `hue` between min and max? Hue is a circle from 0 to 360, and red
// sits on both sides of 0 (340 to 25), so that case is handled too.
bool ColorSorter::in_range(float hue, float min, float max) {
  if (min <= max) return hue >= min && hue <= max;
  return hue >= min || hue <= max;   // the range wraps around 0
}

// What color is in front of the sensor right now?
Color ColorSorter::seen() {
  if (use_proximity && !sensor_->isNearObject()) return NO_COLOR;
  float hue = sensor_->hue();
  if (in_range(hue, red_hue_min, red_hue_max)) return RED;
  if (in_range(hue, blue_hue_min, blue_hue_max)) return BLUE;
  return NO_COLOR;
}

// The three ways to throw a piece out. Each one stores a "start" action
// and a "finish" action for update() to run.
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

// Turns the sensor light on at start-up so colors read the same in any room.
void ColorSorter::init() {
  if (light_power > 0) {
    sensor_->setLightPower(light_power, vex::percent);
    sensor_->setLight(vex::ledState::on);
  }
}

// Puts the ejector back (retracts the piston).
void ColorSorter::end_eject() {
  if (!ejecting_) return;
  ejecting_ = false;
  if (finish_) finish_();
}

// Between match periods: forget any waiting eject and put the ejector back.
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

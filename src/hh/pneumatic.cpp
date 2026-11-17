#include "vex.h"
#include <new>

namespace hh {

Pneumatic::Pneumatic(vex::triport::port& port, bool start_extended)
    : a_(port), extended_(start_extended) {
  name = "Piston";
}

Pneumatic::Pneumatic(vex::triport::port& port_a, vex::triport::port& port_b, bool start_extended)
    : a_(port_a), extended_(start_extended) {
  name = "Piston";
  b_ = new (b_storage_) vex::digital_out(port_b);
}

void Pneumatic::apply() {
  bool output = reversed ? !extended_ : extended_;
  a_.set(output);
  if (b_ != nullptr) b_->set(output);
}

void Pneumatic::set(bool extended) {
  extended_ = extended;
  pulse_end_ = 0;
  apply();
}

void Pneumatic::extend() { set(true); }

void Pneumatic::retract() { set(false); }

void Pneumatic::toggle() { set(!extended_); }

void Pneumatic::pulse(int ms) {
  set(true);
  pulse_end_ = now_ms() + ms;
}

void Pneumatic::bind_toggle(Button button) {
  on_press(button, [this] { toggle(); });
}

void Pneumatic::bind_hold(Button button) {
  while_held(button, [this] { extend(); }, [this] { retract(); });
}

void Pneumatic::init() { apply(); }

void Pneumatic::update() {
  if (pulse_end_ != 0 && now_ms() >= pulse_end_) {
    set(false);
  }
}

}  // namespace hh

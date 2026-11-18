// hh::Pneumatic: one piston, or two that always move together.
//
// A piston is driven by a solenoid on a 3-wire port. The port is either on
// (air in) or off, which is all digital_out.set(true / false) does.

#include "vex.h"
#include <new>

namespace hh {

Pneumatic::Pneumatic(vex::triport::port& port, bool start_extended)
    : a_(port), extended_(start_extended) {
  name = "Piston";
}

// Two solenoids. The second digital_out is built inside b_storage_ (a
// block of memory that is part of this object) only when it is needed,
// because a digital_out can't exist without a port.
Pneumatic::Pneumatic(vex::triport::port& port_a, vex::triport::port& port_b, bool start_extended)
    : a_(port_a), extended_(start_extended) {
  name = "Piston";
  b_ = new (b_storage_) vex::digital_out(port_b);
}

// Sends the current state to the solenoid(s), flipped if `reversed` is set.
void Pneumatic::apply() {
  bool output = reversed ? !extended_ : extended_;
  a_.set(output);
  if (b_ != nullptr) b_->set(output);
}

// Every command ends up here. A new command cancels a pulse in progress.
void Pneumatic::set(bool extended) {
  extended_ = extended;
  pulse_end_ = 0;
  apply();
}

void Pneumatic::extend() { set(true); }

void Pneumatic::retract() { set(false); }

void Pneumatic::toggle() { set(!extended_); }

// Extends now. update() retracts it when the time is up.
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

// At start-up, put the piston in its starting state.
void Pneumatic::init() { apply(); }

// Every 10 ms: end a pulse whose time is up.
void Pneumatic::update() {
  if (pulse_end_ != 0 && now_ms() >= pulse_end_) {
    set(false);
  }
}

}  // namespace hh

#pragma once
#include <functional>
#include "v5.h"
#include "v5_vcs.h"

namespace hh {

/**
 * A controller button, optionally combined with a "shift" button that must be
 * held at the same time. You rarely write this type yourself: anywhere a
 * Button is expected you can pass Controller.ButtonR1 directly, or
 * hh::shifted(Controller.ButtonL2, Controller.ButtonR1) for a shift combo.
 */
struct Button {
  const vex::controller::button* button;
  const vex::controller::button* shift;

  Button(const vex::controller::button& b) : button(&b), shift(nullptr) {}
  Button(const vex::controller::button& shift_key, const vex::controller::button& b)
      : button(&b), shift(&shift_key) {}

  /**
   * True while the button is held. A plain button reads as released while the
   * shift key of one of its own shift combos is held, so R1 and L2+R1 can do
   * different things.
   */
  bool pressing() const;
};

/** `button`, but only while `shift_key` is also held. */
Button shifted(const vex::controller::button& shift_key, const vex::controller::button& button);

typedef std::function<void()> Action;

/** Runs `action` once each time the button is pressed. */
void on_press(Button button, Action action);
/** Runs `action` once each time the button is released. */
void on_release(Button button, Action action);
/** Runs `pressed` when the button goes down and `released` when it comes up. */
void while_held(Button button, Action pressed, Action released);
/** Alternates between `on` and `off` each time the button is pressed. */
void toggle(Button button, Action on, Action off);

/** Checks every binding. hh::driver::update() calls this for you. */
void update_buttons();

}  // namespace hh

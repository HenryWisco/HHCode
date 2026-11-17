#pragma once
#include "vex.h"

namespace hh {

/**
 * Button bindings connect a controller button to an action in one line.
 * Set them up once in setup_controls() in main.cpp. They only do
 * anything during driver control.
 *
 * The [] { ... } part is the action. Put what should happen between
 * the curly braces.
 *
 * Runs the first action the whole time the button is held, and the
 * second action once when it's let go:
 *   hh::bind_hold(Controller1.ButtonR1, [] { intake.spin(); }, [] { intake.stop(); });
 *
 * Runs the action once each time the button is pressed:
 *   hh::bind_press(Controller1.ButtonA, [] { claw.toggle(); });
 *
 * Switches between two actions each time the button is pressed:
 *   hh::bind_toggle(Controller1.ButtonX, [] { intake.spin(); }, [] { intake.stop(); });
 */

void bind_hold(const controller::button &button, void (*while_held)(), void (*on_release)());
void bind_press(const controller::button &button, void (*on_press)());
void bind_toggle(const controller::button &button, void (*turn_on)(), void (*turn_off)());

/** Checks every bound button. Call this once per loop in usercontrol(). */
void run_bindings();

/** Removes every binding. */
void clear_bindings();

}

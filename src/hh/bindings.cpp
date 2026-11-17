#include "vex.h"

namespace hh {

enum binding_type {BIND_HOLD, BIND_PRESS, BIND_TOGGLE};

struct binding
{
  const controller::button *button;
  binding_type type;
  void (*first_action)();
  void (*second_action)();
  bool was_pressed;
  bool toggled_on;
};

static const int max_bindings = 32;
static binding bindings[max_bindings];
static int binding_count = 0;

static void add_binding(const controller::button &button, binding_type type, void (*first_action)(), void (*second_action)()){
  if (binding_count >= max_bindings){
    printf("hh: too many button bindings, the max is %d\n", max_bindings);
    return;
  }
  bindings[binding_count] = {&button, type, first_action, second_action, false, false};
  binding_count++;
}

/**
 * Runs one action the whole time a button is held, and another
 * action once when it's released.
 *
 * @param button The button, i.e. Controller1.ButtonR1.
 * @param while_held Action to run every loop while the button is down.
 * @param on_release Action to run once when the button comes up.
 */

void bind_hold(const controller::button &button, void (*while_held)(), void (*on_release)()){
  add_binding(button, BIND_HOLD, while_held, on_release);
}

/**
 * Runs an action once each time a button is pressed.
 *
 * @param button The button, i.e. Controller1.ButtonA.
 * @param on_press Action to run once per press.
 */

void bind_press(const controller::button &button, void (*on_press)()){
  add_binding(button, BIND_PRESS, on_press, nullptr);
}

/**
 * Switches between two actions each time a button is pressed.
 * The first press runs turn_on, the next runs turn_off, and so on.
 *
 * @param button The button, i.e. Controller1.ButtonX.
 * @param turn_on Action for the first, third, fifth... press.
 * @param turn_off Action for the second, fourth, sixth... press.
 */

void bind_toggle(const controller::button &button, void (*turn_on)(), void (*turn_off)()){
  add_binding(button, BIND_TOGGLE, turn_on, turn_off);
}

/**
 * Checks every bound button and runs its action if needed.
 * Call once per loop in usercontrol().
 */

void run_bindings(){
  for (int i = 0; i < binding_count; i++){
    binding &b = bindings[i];
    bool pressed = b.button->pressing();
    bool just_pressed = pressed && !b.was_pressed;
    bool just_released = !pressed && b.was_pressed;

    if (b.type == BIND_HOLD){
      if (pressed && b.first_action != nullptr) b.first_action();
      if (just_released && b.second_action != nullptr) b.second_action();
    }
    if (b.type == BIND_PRESS){
      if (just_pressed && b.first_action != nullptr) b.first_action();
    }
    if (b.type == BIND_TOGGLE && just_pressed){
      b.toggled_on = !b.toggled_on;
      if (b.toggled_on && b.first_action != nullptr) b.first_action();
      if (!b.toggled_on && b.second_action != nullptr) b.second_action();
    }

    b.was_pressed = pressed;
  }
}

void clear_bindings(){
  binding_count = 0;
}

}

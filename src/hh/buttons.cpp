// Button bindings: "when this button is pressed, do that".
//
// configure_robot() calls on_press(), while_held() and toggle() to build a
// list of bindings. During driver control, update_buttons() runs every
// 20 ms, looks at each button in the list, and runs the action when the
// button has just gone down or just come up.

#include "vex.h"
#include <vector>

namespace hh {

// The four ways a button can be bound.
enum BindingKind { PRESS, RELEASE, HELD, TOGGLE };

// One line from configure_robot(), remembered.
//   first / second   the actions (press and release, or on and off)
//   last             whether the button was down the last time we looked
//   toggled          for TOGGLE: whether it is currently "on"
struct Binding {
  Button button;
  BindingKind kind;
  Action first;
  Action second;
  bool last;
  bool toggled;
};

// The list of every binding. It lives inside a function so it is ready
// even if something binds a button very early in start-up.
static std::vector<Binding>& bindings() {
  static std::vector<Binding> list;
  return list;
}

// Every (shift key, button) combo that has a binding.
static std::vector<Button>& combos() {
  static std::vector<Button> list;
  return list;
}

// Adds one binding to the list. Shift combos are also noted in combos().
static void add(Button button, BindingKind kind, Action first, Action second) {
  if (button.shift != nullptr) combos().push_back(button);
  bindings().push_back({button, kind, first, second, false, false});
}

// Is this button "down" as far as its binding is concerned?
//   - A shift combo (L2 + R1) needs both buttons held.
//   - A plain button (R1) does not count while the shift key of one of
//     its own combos is held, so R1 and L2 + R1 can do different jobs.
bool Button::pressing() const {
  if (!button->pressing()) return false;
  if (shift != nullptr) return shift->pressing();
  for (const Button& combo : combos()) {
    if (combo.button == button && combo.shift->pressing()) return false;
  }
  return true;
}

Button shifted(const vex::controller::button& shift_key, const vex::controller::button& button) {
  return Button(shift_key, button);
}

// The four functions teams call. Each just adds to the list.
void on_press(Button button, Action action) { add(button, PRESS, action, nullptr); }

void on_release(Button button, Action action) { add(button, RELEASE, action, nullptr); }

void while_held(Button button, Action pressed, Action released) {
  add(button, HELD, pressed, released);
}

void toggle(Button button, Action on, Action off) { add(button, TOGGLE, on, off); }

// Runs every loop of driver control. "pressed" and "released" are true
// for one loop only: the loop in which the button changed.
void update_buttons() {
  for (Binding& b : bindings()) {
    bool now = b.button.pressing();
    bool pressed = now && !b.last;
    bool released = !now && b.last;
    b.last = now;

    switch (b.kind) {
      case PRESS:
        if (pressed && b.first) b.first();
        break;
      case RELEASE:
        if (released && b.first) b.first();
        break;
      case HELD:
        if (pressed && b.first) b.first();
        if (released && b.second) b.second();
        break;
      case TOGGLE:
        if (pressed) {
          b.toggled = !b.toggled;
          if (b.toggled && b.first) b.first();
          if (!b.toggled && b.second) b.second();
        }
        break;
    }
  }
}

}  // namespace hh

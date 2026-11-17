#include "vex.h"
#include <vector>

namespace hh {

enum BindingKind { PRESS, RELEASE, HELD, TOGGLE };

struct Binding {
  Button button;
  BindingKind kind;
  Action first;
  Action second;
  bool last;
  bool toggled;
};

static std::vector<Binding>& bindings() {
  static std::vector<Binding> list;
  return list;
}

// Every (shift key, button) combo that has a binding.
static std::vector<Button>& combos() {
  static std::vector<Button> list;
  return list;
}

static void add(Button button, BindingKind kind, Action first, Action second) {
  if (button.shift != nullptr) combos().push_back(button);
  bindings().push_back({button, kind, first, second, false, false});
}

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

void on_press(Button button, Action action) { add(button, PRESS, action, nullptr); }

void on_release(Button button, Action action) { add(button, RELEASE, action, nullptr); }

void while_held(Button button, Action pressed, Action released) {
  add(button, HELD, pressed, released);
}

void toggle(Button button, Action on, Action off) { add(button, TOGGLE, on, off); }

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

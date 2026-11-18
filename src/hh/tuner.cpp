#include "vex.h"

namespace hh {
namespace tuner {

bool enabled = false;

struct Item {
  const char* name;
  float* value;
  float step;
};

static Item items[] = {
  {"Drive kP", &chassis.drive_kp, 0.05},
  {"Drive kI", &chassis.drive_ki, 0.005},
  {"Drive kD", &chassis.drive_kd, 0.5},
  {"Heading kP", &chassis.heading_kp, 0.02},
  {"Heading kI", &chassis.heading_ki, 0.005},
  {"Heading kD", &chassis.heading_kd, 0.1},
  {"Turn kP", &chassis.turn_kp, 0.02},
  {"Turn kI", &chassis.turn_ki, 0.005},
  {"Turn kD", &chassis.turn_kd, 0.1},
  {"Swing kP", &chassis.swing_kp, 0.02},
  {"Swing kI", &chassis.swing_ki, 0.001},
  {"Swing kD", &chassis.swing_kd, 0.1},
  {"Drive slew", &chassis.drive_slew, 5},
  {"Turn slew", &chassis.turn_slew, 5},
};
static const int ITEM_COUNT = sizeof(items) / sizeof(items[0]);

static int current = 0;
static bool running = false;
static bool shown = false;
static vex::task test_task;

static int run_test() {
  selector::run_selected();
  chassis.wait();
  chassis.drive_stop(vex::brakeType::coast);
  running = false;
  shown = false;
  return 0;
}

static void stop_test() {
  if (!running) return;
  test_task.stop();
  chassis.cancel_motion();
  chassis.drive_stop(vex::brakeType::coast);
  chassis.set_mirrored(false);
  running = false;
}

void reset() {
  running = false;
  shown = false;
}

static void show() {
  Controller.Screen.clearScreen();
  Controller.Screen.setCursor(1, 1);
  Controller.Screen.print("%s", items[current].name);
  Controller.Screen.setCursor(2, 1);
  Controller.Screen.print("%.3f", *items[current].value);
  Controller.Screen.setCursor(3, 1);
  Controller.Screen.print(running ? "RUNNING  B:stop" : "A:run X:print");
}

static void print_all() {
  printf("\n// Tuned constants. Paste into default_constants() in src/autons/constants.cpp\n");
  printf("chassis.set_drive_constants(%.1f, %.3f, %.3f, %.3f, %.1f);\n", chassis.drive_max_voltage,
         chassis.drive_kp, chassis.drive_ki, chassis.drive_kd, chassis.drive_starti);
  printf("chassis.set_heading_constants(%.1f, %.3f, %.3f, %.3f, %.1f);\n", chassis.heading_max_voltage,
         chassis.heading_kp, chassis.heading_ki, chassis.heading_kd, chassis.heading_starti);
  printf("chassis.set_turn_constants(%.1f, %.3f, %.3f, %.3f, %.1f);\n", chassis.turn_max_voltage,
         chassis.turn_kp, chassis.turn_ki, chassis.turn_kd, chassis.turn_starti);
  printf("chassis.set_swing_constants(%.1f, %.3f, %.3f, %.3f, %.1f);\n", chassis.swing_max_voltage,
         chassis.swing_kp, chassis.swing_ki, chassis.swing_kd, chassis.swing_starti);
  printf("chassis.drive_slew = %.1f;\n", chassis.drive_slew);
  printf("chassis.turn_slew = %.1f;\n", chassis.turn_slew);
}

bool update() {
  static bool last[7] = {false, false, false, false, false, false, false};
  bool now[7] = {Controller.ButtonUp.pressing(),   Controller.ButtonDown.pressing(),
                 Controller.ButtonLeft.pressing(), Controller.ButtonRight.pressing(),
                 Controller.ButtonA.pressing(),    Controller.ButtonB.pressing(),
                 Controller.ButtonX.pressing()};
  bool pressed[7];
  for (int i = 0; i < 7; i++) {
    pressed[i] = now[i] && !last[i];
    last[i] = now[i];
  }

  bool changed = !shown;
  if (pressed[0]) { current = (current + ITEM_COUNT - 1) % ITEM_COUNT; changed = true; }
  if (pressed[1]) { current = (current + 1) % ITEM_COUNT; changed = true; }
  if (pressed[2] || pressed[3]) {
    float value = *items[current].value + (pressed[3] ? items[current].step : -items[current].step);
    if (value < 0) value = 0;
    *items[current].value = value;
    changed = true;
  }
  if (pressed[4] && !running) {
    running = true;
    changed = true;
    test_task = vex::task(run_test);
  }
  if (pressed[5] && running) {
    stop_test();
    changed = true;
  }
  if (pressed[6]) print_all();

  if (changed) {
    shown = true;
    show();
  }
  return running;
}

}  // namespace tuner
}  // namespace hh

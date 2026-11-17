#include "vex.h"
#include <vector>

namespace hh {

// A plain array (not a vector) so subsystems declared as globals can register
// themselves safely before main() starts.
static const int MAX_SUBSYSTEMS = 32;
static Subsystem* subsystem_list[MAX_SUBSYSTEMS];
static int subsystem_total = 0;

Subsystem::Subsystem() {
  if (subsystem_total < MAX_SUBSYSTEMS) {
    subsystem_list[subsystem_total++] = this;
  }
}

int subsystem_count() { return subsystem_total; }

Subsystem* subsystem_at(int index) {
  if (index < 0 || index >= subsystem_total) return nullptr;
  return subsystem_list[index];
}

struct Trigger {
  std::function<bool()> condition;
  std::function<void()> action;
  bool last;
};

static std::vector<Trigger>& triggers() {
  static std::vector<Trigger> list;
  return list;
}

void when(std::function<bool()> condition, std::function<void()> action) {
  triggers().push_back({condition, action, false});
}

uint32_t now_ms() { return vex::timer::system(); }

bool robot_enabled() { return Competition.isEnabled(); }

static int background_task() {
  bool was_enabled = false;
  while (true) {
    bool enabled = robot_enabled();
    if (was_enabled && !enabled) {
      // The Brain stops every motor when the robot is disabled. Let each
      // mechanism forget what it was doing so nothing restarts by itself.
      for (int i = 0; i < subsystem_total; i++) {
        subsystem_list[i]->on_disable();
      }
    }
    was_enabled = enabled;

    if (enabled) {
      for (int i = 0; i < subsystem_total; i++) {
        subsystem_list[i]->update();
      }
      for (auto& trigger : triggers()) {
        bool now = trigger.condition();
        if (now && !trigger.last) trigger.action();
        trigger.last = now;
      }
    }
    driver::check_overheat();
    vex::task::sleep(10);
  }
  return 0;
}

static void calibrate_inertial() {
  if (!chassis.Gyro.installed()) return;
  Brain.Screen.clearScreen();
  Brain.Screen.setFont(vex::fontType::mono30);
  Brain.Screen.printAt(20, 100, "Calibrating inertial sensor");
  Brain.Screen.printAt(20, 140, "Don't move the robot!");
  Brain.Screen.setFont(vex::fontType::mono20);
  chassis.Gyro.calibrate();
  uint32_t start = now_ms();
  while (chassis.Gyro.isCalibrating() && now_ms() - start < 5000) {
    vex::task::sleep(20);
  }
  Brain.Screen.clearScreen();
}

void init() {
  static bool started = false;
  if (started) return;
  started = true;

  calibrate_inertial();
  for (int i = 0; i < subsystem_total; i++) {
    subsystem_list[i]->init();
  }
  driver::load_saved_curves();
  static vex::task background(background_task);
  selector::start();
  graph::start();
}

}  // namespace hh

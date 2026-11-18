// Driver profiles: a named set of driver settings for each driver.
//
// start() takes a snapshot of the settings configure_robot() made (the
// "base"). Picking a profile puts the base back and then applies that
// profile's changes on top.

#include "vex.h"

namespace hh {
namespace profiles {

struct Profile {
  const char* name;
  std::function<void()> settings;   // empty for a profile that changes nothing
};

static const int MAX_PROFILES = 8;
static Profile list[MAX_PROFILES];
static int total = 0;
static int current = 0;

static const char* SAVE_FILE = "hh_driver.txt";

// The driver settings exactly as configure_robot() left them. Every profile
// starts from these.
struct Settings {
  DriveStyle style;
  float stick_deadband, throttle_curve, turn_curve, turn_scale, strafe_scale, active_brake_kp;
};
static Settings base;
static bool started = false;

void add(const char* name, std::function<void()> settings) {
  if (total < MAX_PROFILES) list[total++] = {name, settings};
}

void add(const char* name) { add(name, std::function<void()>()); }

void add(const char* name, DriveStyle style, float throttle_curve, float turn_curve) {
  add(name, [style, throttle_curve, turn_curve] {
    driver::style = style;
    driver::throttle_curve = throttle_curve;
    driver::turn_curve = turn_curve;
  });
}

int count() { return total; }

int selected() { return current; }

const char* selected_name() { return total > 0 ? list[current].name : ""; }

// Puts the current profile's settings into hh::driver.
static void apply() {
  if (total == 0 || !started) return;
  driver::style = base.style;
  driver::stick_deadband = base.stick_deadband;
  driver::throttle_curve = base.throttle_curve;
  driver::turn_curve = base.turn_curve;
  driver::turn_scale = base.turn_scale;
  driver::strafe_scale = base.strafe_scale;
  driver::active_brake_kp = base.active_brake_kp;
  if (list[current].settings) list[current].settings();
  // Curves this driver tuned live (hh::driver::curve_tuning) win over the
  // ones written in the code.
  driver::load_saved_curves();
}

void select(int index) {
  if (total == 0) return;
  current = ((index % total) + total) % total;
  apply();
  // The name is saved rather than the number, so reordering the list doesn't
  // silently change who is driving.
  if (Brain.SDcard.isInserted()) {
    const char* name = list[current].name;
    Brain.SDcard.savefile(SAVE_FILE, (uint8_t*)name, strlen(name));
  }
}

void next() { select(current + 1); }

void previous() { select(current - 1); }

void start() {
  base = {driver::style, driver::stick_deadband, driver::throttle_curve, driver::turn_curve,
          driver::turn_scale, driver::strafe_scale, driver::active_brake_kp};
  started = true;
  if (total == 0) return;

  if (Brain.SDcard.isInserted() && Brain.SDcard.exists(SAVE_FILE)) {
    char name[64] = {0};
    Brain.SDcard.loadfile(SAVE_FILE, (uint8_t*)name, sizeof(name) - 1);
    for (int i = 0; i < total; i++) {
      if (strcmp(list[i].name, name) == 0) current = i;
    }
  }
  apply();
}

}  // namespace profiles
}  // namespace hh

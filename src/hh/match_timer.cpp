#include "vex.h"

namespace hh {
namespace match_timer {

float driver_length = 105;
float endgame_time = 30;
bool show_on_controller = false;

struct Warning {
  float seconds_left;
  const char* pattern;
  bool done;          // already buzzed during this run of the clock
};

// The two built-in warnings are here from the start. warn_at() adds more.
static const int MAX_WARNINGS = 8;
static Warning warnings[MAX_WARNINGS] = {{30, "-", false}, {15, "- -", false}};
static int warning_total = 2;

static bool counting = false;
static uint32_t start_time = 0;

void warn_at(float seconds_left, const char* pattern) {
  if (warning_total < MAX_WARNINGS) warnings[warning_total++] = {seconds_left, pattern, false};
}

void clear_warnings() { warning_total = 0; }

void start() {
  counting = true;
  start_time = now_ms();
  for (int i = 0; i < warning_total; i++) warnings[i].done = false;
}

void stop() { counting = false; }

bool running() { return counting; }

float time_left() {
  if (!counting) return -1;
  float left = driver_length - (now_ms() - start_time) / 1000.0;
  return left > 0 ? left : 0;
}

bool in_endgame() { return !counting || time_left() <= endgame_time; }

Action endgame_only(Action action) {
  return [action] {
    if (in_endgame()) {
      if (action) action();
    } else {
      Controller.rumble(".");   // "not yet"
    }
  };
}

void update() {
  // Start the clock when driver control begins in a real match. Without a
  // competition switch or field, driver control runs from the moment the
  // program starts, so there is no match to time unless start() is called.
  static bool was_in_match_driver = false;
  bool on_field = Competition.isCompetitionSwitch() || Competition.isFieldControl();
  bool in_match_driver = on_field && Competition.isEnabled() && Competition.isDriverControl();
  if (in_match_driver && !was_in_match_driver) start();
  was_in_match_driver = in_match_driver;

  if (!counting) return;

  // Disabled means the period is over (or the match was stopped).
  if (!Competition.isEnabled()) {
    stop();
    return;
  }

  float left = time_left();
  for (int i = 0; i < warning_total; i++) {
    if (!warnings[i].done && left <= warnings[i].seconds_left) {
      warnings[i].done = true;
      // A warning for a time longer than the whole period would go off the
      // moment the clock starts. Skip those.
      if (warnings[i].seconds_left < driver_length) Controller.rumble(warnings[i].pattern);
    }
  }

  // Writing to the controller screen is slow, so only do it once a second.
  static int last_shown = -1;
  int whole_seconds = (int)left;
  if (show_on_controller && !tuner::enabled && whole_seconds != last_shown) {
    last_shown = whole_seconds;
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Time %d:%02d      ", whole_seconds / 60, whole_seconds % 60);
  }

  if (left <= 0) stop();
}

}  // namespace match_timer
}  // namespace hh

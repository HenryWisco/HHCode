#pragma once
#include "hh/buttons.h"

namespace hh {

/**
 * Match timer: buzzes the controller when driver control is nearly over, and
 * can keep an endgame mechanism (a hang, for example) locked until the end
 * of the match.
 *
 * It starts by itself when driver control begins on a competition switch or
 * a real field, and buzzes with 30 and 15 seconds left. There is nothing to
 * set up for that. For practice without a switch, start it from a button:
 *
 *   hh::on_press(Controller.ButtonUp, [] { hh::match_timer::start(); });
 *
 * All of these go in configure_robot().
 */
namespace match_timer {

/**
 * How long driver control lasts, in seconds. 105 is a normal match (1:45).
 * Use 60 for a driver skills run.
 */
extern float driver_length;

/** The last this-many seconds of driver control count as the endgame. */
extern float endgame_time;

/** Turn on to show the time left on the second line of the controller. */
extern bool show_on_controller;

/**
 * Adds a buzz with `seconds_left` on the clock. `pattern` is made of dots
 * (short buzz), dashes (long buzz) and spaces (pause). Up to 8 warnings.
 *
 *   hh::match_timer::warn_at(10, "...");
 */
void warn_at(float seconds_left, const char* pattern = "-");

/** Removes every warning, including the built-in ones at 30 and 15 seconds. */
void clear_warnings();

/** Starts the clock now. For practice without a competition switch. */
void start();
/** Stops the clock. */
void stop();
/** True while the clock is counting down. */
bool running();
/** Seconds of driver control left, or -1 when the clock isn't running. */
float time_left();

/**
 * True in the last endgame_time seconds. Also true whenever the clock isn't
 * running, so endgame mechanisms still work in practice and in the pits.
 */
bool in_endgame();

/**
 * Wraps a button action so it only works in the endgame. Pressing the button
 * too early gives one short buzz and does nothing.
 *
 *   hh::on_press(Controller.ButtonX, hh::match_timer::endgame_only([] { hang.extend(); }));
 */
Action endgame_only(Action action);

/** Checks the clock. The HHCode background task calls this for you. */
void update();

}  // namespace match_timer
}  // namespace hh

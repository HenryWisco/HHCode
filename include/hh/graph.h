#pragma once

namespace hh {

/**
 * PID graph. Draws the error and the motor voltage of the latest chassis
 * motion on the Brain screen, so you can see overshoot and wobble instead of
 * guessing. Made to be used with the tuner (hh::tuner).
 *
 * Turn it on with  hh::graph::enabled = true;  in configure_robot(). The graph
 * takes over the Brain screen whenever the chassis runs a motion. Tap the
 * screen to get the auton selector back. Turn it off for competition.
 */
namespace graph {

extern bool enabled;

// Numbers from the latest motion, for printing or for your own checks.
/** How long the motion took, in ms. */
float duration_ms();
/** The error when the motion started (inches or degrees). */
float start_error();
/** The error when it ended. */
float final_error();
/** How far it went past the target. 0 if it never crossed it. */
float overshoot();
/** How many points are stored. */
int samples();

/** True while the graph is using the Brain screen. */
bool owns_screen();

// Called by the chassis; you don't need these.
void begin(const char* label, const char* units);
void record(float error, float output);
/** Starts the drawing task. hh::init() calls this for you. */
void start();

}  // namespace graph
}  // namespace hh

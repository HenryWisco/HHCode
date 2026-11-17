#pragma once

namespace hh {

/**
 * Live PID tuner. Change the chassis constants from the controller and re-run
 * an auton straight away, without downloading again.
 *
 * Turn it on with  hh::tuner::enabled = true;  in configure_robot(). While it
 * is on, your own button bindings are paused and the controller does this:
 *
 *   Up / Down      pick a constant (Drive kP, Drive kD, Turn kP, ...)
 *   Left / Right   lower / raise it
 *   A              run the auton chosen on the Brain screen
 *   B              stop the auton
 *   X              print every constant to the VS Code terminal, ready to
 *                  paste into src/autons/constants.cpp
 *
 * The joysticks still drive, so you can put the robot back between runs.
 * Changes are lost when the program restarts: copy them into constants.cpp.
 * Turn the tuner off for competition.
 */
namespace tuner {

extern bool enabled;

/** Runs the tuner. Returns true while a test auton is running. */
bool update();
/** Forgets any test run. Called when driver control starts. */
void reset();

}  // namespace tuner
}  // namespace hh

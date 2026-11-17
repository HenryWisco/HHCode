#pragma once

namespace hh {

/**
 * Auton selector. Shows on the Brain screen: tap the left half for the
 * previous auton, the right half for the next one. While the robot is disabled
 * by a competition switch or field control, the controller's Left/Right arrows
 * work too. The choice is saved to the SD card (if there is one) and comes
 * back the next time the program starts.
 */
namespace selector {

/** Turn off to free the Brain screen for your own printing. */
extern bool show_on_brain;
/** Turn off to stop the controller arrows from changing the auton. */
extern bool use_controller;

/**
 * Adds an auton to the list. Call from register_autons(). The first one added
 * is the default.
 *
 *   hh::selector::add("Red Left", "4 rings, touches ladder", red_left);
 */
void add(const char* name, const char* description, void (*routine)());

/** Runs the chosen auton. Call from autonomous(). */
void run_selected();

void next();
void previous();
int selected();
int count();
const char* selected_name();

/** Starts the selector. hh::init() calls this for you. */
void start();

}  // namespace selector
}  // namespace hh

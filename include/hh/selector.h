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

/**
 * Adds one routine twice: as written, and mirrored for the other side of the
 * field. Write and tune it once, on the side named first.
 *
 *   hh::selector::add_mirrored("Red Left", "Blue Right", "4 rings, touches ladder", red_left);
 *
 * In the mirrored version every chassis motion is flipped left-to-right:
 * turn_to_angle(90) turns left instead of right, drive_to_point(24, 36) goes
 * to (-24, 36), strafes and swings swap sides. Mechanisms are not changed; if
 * a routine uses a left-side wing, check hh::selector::mirrored() and use the
 * right-side one instead.
 */
void add_mirrored(const char* name, const char* mirrored_name, const char* description, void (*routine)());

/** True while a mirrored auton is running. */
bool mirrored();

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

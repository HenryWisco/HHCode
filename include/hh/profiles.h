#pragma once
#include <functional>
#include "hh/driver.h"

namespace hh {

/**
 * Driver profiles: each driver keeps their own stick layout and joystick
 * curves, and picks their name on the Brain screen before driving.
 *
 *   // in configure_robot(), after the hh::driver settings:
 *   hh::profiles::add("Alex");                           // the settings above, unchanged
 *   hh::profiles::add("Sam", hh::TANK_STICKS, 3, 3);     // layout, throttle curve, turn curve
 *   hh::profiles::add("Jo", [] {                         // anything at all
 *     hh::driver::style = hh::LEFT_ARCADE;
 *     hh::driver::turn_scale = 0.8;
 *   });
 *
 * To pick a driver, tap "Driver: ..." on the Brain's auton screen. While the
 * robot is disabled on a field, the controller's Up and Down arrows work too.
 * The choice is saved to the SD card. The first profile added is the default.
 *
 * Every profile starts from the settings in configure_robot() and changes
 * only what it says, so one driver's settings never leak into another's.
 * Profiles change how the sticks drive. Button bindings are the same for
 * every driver. With no profiles added, none of this shows up.
 */
namespace profiles {

/** A driver who uses the settings in configure_robot() as they are. */
void add(const char* name);
/** A driver with their own stick layout and curves. */
void add(const char* name, DriveStyle style, float throttle_curve = 0, float turn_curve = 0);
/** A driver with any settings: `settings` runs each time they are picked. */
void add(const char* name, std::function<void()> settings);

/** How many profiles there are. */
int count();
/** Which profile is in use (0 is the first one added). */
int selected();
/** The name of the profile in use, or "" when there are none. */
const char* selected_name();
/** Switches to a profile and saves the choice. */
void select(int index);
/** Switches to the next profile, going back to the first after the last. */
void next();
/** Switches to the previous profile. */
void previous();

/** Remembers the base settings and loads the saved choice. hh::init() calls this. */
void start();

}  // namespace profiles
}  // namespace hh

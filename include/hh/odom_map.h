#pragma once

namespace hh {

/**
 * Odometry map. Shows where the robot thinks it is: a top-down picture of
 * the field with the robot, the trail it has left, and its X, Y and heading
 * as numbers.
 *
 * Tap "Map>" in the title bar of the auton screen to open it, and tap again
 * to go back. If it is open when autonomous starts, it stays open so you can
 * watch the routine. It is also the easy way to find coordinates for an
 * auton: push the robot to a spot by hand and read the numbers.
 *
 * Needs a chassis setup with odometry (anything except ZERO_TRACKER_NO_ODOM).
 */
namespace odom_map {

/** How much of the field the map shows, in inches. A full field is 144. */
extern float field_size;
/**
 * The odometry coordinates at the middle of the map. With the default (0, 0),
 * X and Y each run from -72 to 72. If your autons start at (0, 0) in a corner
 * of the field, set these to 72 and 72.
 */
extern float center_x;
extern float center_y;

/** Forgets the trail. */
void clear_trail();
/** How many points are in the trail. */
int trail_points();

/** Draws the map on the Brain. The selector calls this for you. */
void draw();
/** Records the trail. The background task calls this for you. */
void update();

}  // namespace odom_map
}  // namespace hh

#pragma once
#include "v5.h"
#include "v5_vcs.h"

namespace hh {

/** Which way a wall sensor points, seen from the robot. */
enum Facing { FRONT, RIGHT, BACK, LEFT };

/**
 * A distance sensor that looks at the field wall. Position tracking slowly
 * drifts during a long auton. A wall sensor measures how far the robot really
 * is from a wall and corrects X or Y, so the rest of the routine stays
 * accurate.
 *
 *   // Section 1 of robot-config.cpp
 *   distance FrontDistance = distance(PORT7);
 *   distance LeftDistance = distance(PORT8);
 *
 *   // Section 3: sensor, which way it points, inches in front of the
 *   // robot's center (negative = behind), inches right of center
 *   // (negative = left)
 *   hh::WallSensor front_wall(FrontDistance, hh::FRONT, 6.5, 0);
 *   hh::WallSensor left_wall(LeftDistance, hh::LEFT, 1.0, -5.5);
 *
 *   // in an auton, while facing roughly square to the walls:
 *   hh::wall_reset();
 *
 * Needs a chassis setup with position tracking and an inertial sensor. The
 * walls are where the odometry map says they are: a 144 inch field with
 * (0, 0) in the middle, unless you change hh::odom_map::field_size,
 * center_x and center_y.
 *
 * A reading is thrown away (and position is left alone) when the sensor sees
 * nothing, when the robot isn't facing close to square to the wall, or when
 * the answer is far from where the robot thinks it is, which usually means
 * a game piece or another robot was in the way.
 */
class WallSensor {
public:
  /**
   * @param sensor          The distance sensor.
   * @param facing          hh::FRONT, hh::RIGHT, hh::BACK or hh::LEFT.
   * @param forward_offset  Inches from the robot's center forward to the
   *                        sensor's face. Negative if it is behind the center.
   * @param right_offset    Inches from the robot's center to the right.
   *                        Negative if it is left of the center.
   *
   * "Center" is the point the chassis tracks: the middle of the drive wheels.
   */
  WallSensor(vex::distance& sensor, Facing facing, float forward_offset, float right_offset);

  /**
   * For a sensor that isn't mounted exactly square: how many degrees
   * clockwise (seen from above) it points away from its facing.
   */
  float angle_offset = 0;

  /** What the sensor reads in inches, or -1 if it sees nothing in range. */
  float inches();

  /**
   * Corrects X or Y from this one sensor. Returns true if it did.
   * hh::wall_reset() does this for every sensor at once.
   */
  bool reset();

  /** Why the last reset() worked or didn't. Shown on the odometry map. */
  const char* status = "not used yet";

  Facing facing() const { return facing_; }

  /**
   * Works out what this sensor says the robot's X or Y is, without changing
   * anything. Returns false (and sets `status`) if the reading can't be used.
   */
  bool measure(bool& is_x, float& coordinate, float& reading);

private:
  vex::distance* sensor_;
  Facing facing_;
  float forward_offset_;
  float right_offset_;
};

/** Settings for wall resets. Change them in configure_robot(). */
namespace walls {

/**
 * The biggest correction allowed, in inches. A reading that would move the
 * robot further than this is thrown away. 0 turns the check off.
 */
extern float max_correction;
/** How far from square to the wall (degrees) the sensor may point. */
extern float max_angle;
/** Readings longer than this (inches) are ignored. */
extern float max_range;

/** Every wall sensor that exists, in the order they were created. */
int sensor_count();
WallSensor* sensor_at(int index);

}  // namespace walls

/**
 * Corrects the robot's position from every wall sensor that has a clear view
 * of a wall. Returns how many of X and Y were corrected: 0, 1 or 2. When two
 * sensors measure the same direction, the one closer to its wall is used.
 */
int wall_reset();

}  // namespace hh

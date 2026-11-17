# Chassis Recipes

Ready-to-copy versions of the `Drive chassis( ... );` block in
`src/robot-config.cpp`. Pick the one that matches your robot, replace the
whole block from `Drive chassis(` through `);` with it, and change the
values marked `CHANGE`.

The motor names (`LeftFront` and so on) are the ones from Step 1 of
`robot-config.cpp`.

Distances are in inches. Tracking wheel "center distance" is measured from
the robot's turning center to the wheel.

## Tank drive, no tracking wheels (start here)

No position tracking. `drive_distance`, `turn_to_angle` and swings all work.

```cpp
Drive chassis(
  ZERO_TRACKER_NO_ODOM,
  motor_group(LeftFront, LeftMiddle, LeftBack),
  motor_group(RightFront, RightMiddle, RightBack),
  PORT10,   // CHANGE: inertial sensor port
  3.25,     // CHANGE: wheel diameter
  0.6,      // CHANGE: gear ratio, motor gear teeth / wheel gear teeth
  360,      // gyro scale
  PORT2, -PORT3, PORT4, -PORT5,   // holonomic only, leave alone
  5, 2.75, -2,                    // forward tracker, not used
  7, -2.75, 5.5                   // sideways tracker, not used
);
```

## Tank drive, position tracking with no tracking wheels

Uses the drive motors and the inertial sensor to track position. Good
enough for `drive_to_point`, but it drifts if the wheels slip.

```cpp
Drive chassis(
  ZERO_TRACKER_ODOM,
  motor_group(LeftFront, LeftMiddle, LeftBack),
  motor_group(RightFront, RightMiddle, RightBack),
  PORT10,   // CHANGE: inertial sensor port
  3.25,     // CHANGE: wheel diameter
  0.6,      // CHANGE: gear ratio
  360,
  PORT2, -PORT3, PORT4, -PORT5,   // holonomic only, leave alone
  5, 2.75,
  6,        // CHANGE: distance from the robot's center to the right side of the drive
  7, -2.75, 5.5                   // sideways tracker, not used
);
```

## Tank drive, two tracking wheels on rotation sensors

The most accurate tank setup. One wheel rolls forward/back, one rolls sideways.

```cpp
Drive chassis(
  TANK_TWO_ROTATION,
  motor_group(LeftFront, LeftMiddle, LeftBack),
  motor_group(RightFront, RightMiddle, RightBack),
  PORT10,   // CHANGE: inertial sensor port
  3.25,     // CHANGE: wheel diameter
  0.6,      // CHANGE: gear ratio
  360,
  PORT2, -PORT3, PORT4, -PORT5,   // holonomic only, leave alone
  PORT11,   // CHANGE: forward tracker rotation sensor port
  2.75,     // CHANGE: forward tracker wheel diameter (make negative if it counts backward)
  -2,       // CHANGE: forward tracker center distance (right of center is positive)
  PORT12,   // CHANGE: sideways tracker rotation sensor port
  -2.75,    // CHANGE: sideways tracker wheel diameter (make negative if it counts backward)
  5.5       // CHANGE: sideways tracker center distance (behind center is positive)
);
```

Only one tracking wheel? Use `TANK_ONE_FORWARD_ROTATION` or
`TANK_ONE_SIDEWAYS_ROTATION` and fill in just that tracker.

Using red 3-wire encoders instead of rotation sensors? Change `ROTATION` to
`ENCODER` in the setup name, and give the port as a number: 1 for three-wire
port A, 2 for B, and so on. An encoder takes two ports side by side, so 1
means A and B.

## X-drive (or mecanum) with two tracking wheels

X-drive needs four motors, one per corner, and tracking wheels for position
tracking.

In Step 1 of `robot-config.cpp`, keep four drive motors:
`LeftFront`, `LeftBack`, `RightFront`, `RightBack`.

```cpp
Drive chassis(
  HOLONOMIC_TWO_ROTATION,
  motor_group(LeftFront, LeftBack),
  motor_group(RightFront, RightBack),
  PORT10,   // CHANGE: inertial sensor port
  3.25,     // CHANGE: wheel diameter
  1,        // CHANGE: gear ratio
  360,
  // CHANGE: the same four drive motors again, as PORTS, by corner.
  // A minus sign reverses that motor. Don't use PORT1 for these.
  // LF       RF
  PORT2,   -PORT3,
  // LB       RB
  PORT4,   -PORT5,
  PORT11,   // CHANGE: forward tracker rotation sensor port
  2.75,     // CHANGE: forward tracker wheel diameter
  -2,       // CHANGE: forward tracker center distance (right of center is positive)
  PORT12,   // CHANGE: sideways tracker rotation sensor port
  -2.75,    // CHANGE: sideways tracker wheel diameter
  5.5       // CHANGE: sideways tracker center distance (behind center is positive)
);
```

Then in `usercontrol()` in `src/main.cpp`, use `chassis.control_holonomic();`.

In autons, `chassis.holonomic_drive_to_pose(x, y, angle);` drives to a spot
while turning to face an angle.

An X-drive with **no** tracking wheels can still use `drive_distance` and
`turn_to_angle`: use the first recipe (`ZERO_TRACKER_NO_ODOM`) with your four
motors in the two motor groups, and keep the four corner ports filled in so
`control_holonomic()` works.

## Three-wire ports and pistons

The chassis always sets up three-wire ports for tracking wheels, even when
you don't use any. With the default values that's ports E, F, G and H, and
with rotation-sensor trackers it can be A and B. Pistons still work on those
ports as long as Step 3 (pistons) stays **below** Step 2 (chassis) in
`robot-config.cpp`.

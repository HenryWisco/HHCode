# Chassis Recipes

Ready-to-copy versions of the `Drive chassis( ... );` block in section 2 of
`src/robot-config.cpp`. Pick the one that matches your robot, replace the
whole block from `Drive chassis(` through `);` with it, and change the
values marked `CHANGE`.

The motor names (`LeftFront` and so on) are whatever you called your motors
in section 1.

Distances are in inches. Tracking wheel "center distance" is measured from
the robot's turning center to the wheel.

Each recipe also says what to set for `hh::driver::drive_type` in
`configure_robot()` (section 4).

## The short way: one line

`hh::Chassis` builds the same chassis from one line. Ports are plain numbers
from 1 to 21, a minus sign means reversed, and you don't create drive motors
in section 1 at all. Replace the whole block from `Drive chassis(` through
`);` with one of these and change the numbers.

```cpp
// Tank: left ports, right ports, inertial port, wheel diameter, cartridge, wheel rpm
Drive chassis = hh::Chassis::tank({1, -2, 3}, {-4, 5, -6}, 10, 3.25, ratio6_1, 450);

// Mecanum or X-drive: left front, right front, left back, right back, then the same
Drive chassis = hh::Chassis::mecanum(1, -2, 3, -4, 10, 4, ratio18_1, 200);
Drive chassis = hh::Chassis::x_drive(1, -2, 3, -4, 10, 3.25, ratio6_1, 300);
```

- **Cartridge:** `ratio36_1` is red (100 rpm), `ratio18_1` is green (200 rpm),
  `ratio6_1` is blue (600 rpm).
- **Wheel rpm** is how fast the wheels turn: cartridge rpm x motor gear teeth
  / wheel gear teeth. Blue motors with a 36-tooth gear driving a 48-tooth gear
  on the wheel is 600 x 36 / 48 = 450. Direct drive is just the cartridge rpm.
- Every drive motor must push the robot forward when it spins forward. If one
  runs backwards, add or remove its minus sign.
- `mecanum(...)` and `x_drive(...)` set `hh::driver::drive_type` for you, and
  they give you position tracking from the drive motors with nothing extra.
  A reversed motor on port 1 is fine here.

Position tracking is added on the end of the line:

```cpp
// Tank, tracking from the drive motors. 11.5 = inches between left and right wheels.
Drive chassis = hh::Chassis::tank({1, 2}, {-3, -4}, 10, 3.25, ratio6_1, 360).odom(11.5);

// Two tracking wheels on rotation sensors: port, wheel diameter, distance from center.
Drive chassis = hh::Chassis::tank({1, 2}, {-3, -4}, 10, 3.25, ratio6_1, 360)
                  .forward_tracker(12, 2.75, -2)     // 2" left of center
                  .sideways_tracker(13, 2.75, 5.5);  // 5.5" behind center
```

A minus sign on a tracker port makes it count the other way. Trackers on
3-wire encoders use `.forward_encoder('A', 2.75, -2)` and
`.sideways_encoder('C', 2.75, 5.5)` (the letter is the first of the encoder's
two ports). `.gyro_scale(357)` sets the gyro scale.

The rest of this page is the long way, which does the same things and matches
the JAR-Template docs.

## Tank drive, no tracking wheels (start here)

No position tracking. `drive_distance`, `turn_to_angle` and swings all work.
`drive_type` is `hh::TANK`.

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
enough for `drive_to_point` and `follow_path`, but it drifts if the wheels
slip. `drive_type` is `hh::TANK`.

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

The most accurate tank setup. One wheel rolls forward/back, one rolls
sideways. `drive_type` is `hh::TANK`.

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

## X-drive or mecanum, no tracking wheels

Four motors, one per corner. Position comes from the drive motors and the
inertial sensor, so `strafe_distance`, `holonomic_drive_to_pose` and
field-centric driving all work without extra sensors. It drifts if the wheels
slip, and mecanum wheels slip sideways: see `chassis.strafe_multiplier` in
[GUIDE.md](GUIDE.md).

`drive_type` is `hh::X_DRIVE` or `hh::MECANUM`, and `style` is
`hh::HOLONOMIC` or `hh::FIELD_CENTRIC`.

```cpp
Drive chassis(
  HOLONOMIC_ZERO_TRACKER_ODOM,
  motor_group(LeftFront, LeftBack),
  motor_group(RightFront, RightBack),
  PORT10,   // CHANGE: inertial sensor port
  3.25,     // CHANGE: wheel diameter
  1,        // CHANGE: gear ratio
  360,
  // CHANGE: the same four drive motors again, as PORTS, by corner.
  // Put a minus sign on the ones that are "true" (reversed) in section 1.
  // Don't plug a reversed one into PORT1: -PORT1 can't be told apart from PORT1.
  // LF       RF
  PORT2,   -PORT3,
  // LB       RB
  PORT4,   -PORT5,
  5, 2.75, 0,                     // forward tracker, not used
  7, -2.75, 0                     // sideways tracker, not used
);
```

Blue or red motors on the wheels also need this line in `configure_robot()`:

```cpp
chassis.set_holonomic_cartridge(ratio6_1);   // ratio36_1 for red
```

## X-drive or mecanum with two tracking wheels

The most accurate holonomic setup. Same `drive_type` and `style` as above.

```cpp
Drive chassis(
  HOLONOMIC_TWO_ROTATION,
  motor_group(LeftFront, LeftBack),
  motor_group(RightFront, RightBack),
  PORT10,   // CHANGE: inertial sensor port
  3.25,     // CHANGE: wheel diameter
  1,        // CHANGE: gear ratio
  360,
  // CHANGE: the four drive motors as PORTS, by corner (see the recipe above).
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

In autons, `chassis.holonomic_drive_to_pose(x, y, angle);` drives to a spot
while turning to face an angle.

## Three-wire ports and pistons

The chassis always sets up three-wire ports for tracking wheels, even when
you don't use any. Which ports depends on the two tracker port numbers in the
recipe (5 and 7 mean E to H; rotation-sensor ports can end up as A and B).
Pistons still work on those ports as long as section 3 (mechanisms) stays
**below** section 2 (chassis) in `robot-config.cpp`.

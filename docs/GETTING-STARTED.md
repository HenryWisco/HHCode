# Getting Started

This walks you from "the template is open in VS Code" to "the robot drives,
the mechanisms work, and an auton runs". If VS Code isn't set up yet, do
[SETUP.md](SETUP.md) first. When you want more than the basics, every feature
is explained in [GUIDE.md](GUIDE.md).

You will edit two places, in this order:

| File | What you do there |
|---|---|
| `src/robot-config.cpp` (and `include/robot-config.h`) | Describe your robot: ports, wheels, mechanisms, buttons |
| `src/autons/` | Write your autons |

Leave the `JAR-Template` and `hh` folders alone. That's the library. You
won't normally need to touch `main.cpp` either.

## 1. Describe your robot

Open `src/robot-config.cpp`. It has four numbered sections with instructions
in the comments.

1. **Motors and sensors.** One line per motor: its port, cartridge color and
   whether it's reversed.
   ```cpp
   motor LeftFront = motor(PORT1, ratio6_1, true);
   ```
2. **The chassis.** Put your drive motors in the two `motor_group()` lines,
   then set the inertial sensor port, wheel size and gear ratio. Leave the
   drive setup as `ZERO_TRACKER_NO_ODOM` for now. Building an X-drive, or using
   tracking wheels? Copy a ready-made chassis from
   [CONFIG-RECIPES.md](CONFIG-RECIPES.md). That page also shows the short
   way: the whole chassis in one line, with no drive motors in section 1.
3. **Mechanisms.** Wrap your other motors and pistons in blocks (table below).
4. **Driver control.** Pick your drivetrain type and stick layout, then give
   each mechanism its buttons.

Every time you add, remove or rename something in sections 1 or 3, make the
matching change in `include/robot-config.h`. It's the same line with `extern`
in front and without the part in brackets.

**Build** (bottom bar) after each step. Fix the first error in the list
before looking at the others.

## 2. The mechanism blocks

| Block | Use it for | Commands |
|---|---|---|
| `hh::Spinner` | Roller intake, conveyor | `spin()`, `spin(60)`, `reverse()`, `stop()` |
| `hh::Positioner` | Lift, arm, wrist | `go_to("high")`, `go_to(45)`, `wait()`, `hold()` |
| `hh::Pneumatic` | Claw, clamp, wings | `extend()`, `retract()`, `toggle()` |
| `hh::Velocity` | Flywheel | `set_rpm(2400)`, `wait_until_ready()`, `stop()` |
| `hh::ColorSorter` | Throwing out the other alliance's pieces | `keep(hh::RED)` |

Making one takes two lines in `robot-config.cpp` and two in `robot-config.h`:

```cpp
// robot-config.cpp
motor IntakeMotor = motor(PORT7, ratio6_1, false);   // section 1
hh::Spinner intake(IntakeMotor);                     // section 3

// robot-config.h
extern motor IntakeMotor;
extern hh::Spinner intake;
```

Things to know:

- **Positioner counts from where it starts.** Wherever the lift is when the
  program starts is 0 degrees. Always start with it all the way down.
- **Positioner doesn't wait.** `lift.go_to("high");` starts the lift and
  moves on, so the robot can drive while it rises. Add `lift.wait();` when the
  next step needs the lift to be there.
- **Piston backward?** If `extend()` retracts it, add `claw.reversed = true;`
  in `configure_robot()`.
- **Mechanisms must stay below the chassis** in `robot-config.cpp` (they
  already are). Don't move section 3 above section 2, or pistons can stop working.
- **Don't name anything `clamp`.** The library already uses that name. Use
  `mogo_clamp` or similar.

## 3. Buttons, lift heights and drive style

All of this goes in `configure_robot()`, at the bottom of `robot-config.cpp`.

```cpp
hh::driver::drive_type = hh::TANK;          // or hh::MECANUM, hh::X_DRIVE
hh::driver::style = hh::SPLIT_ARCADE;       // or hh::TANK_STICKS, and more

intake.bind_hold(Controller.ButtonR1, Controller.ButtonR2);   // R1 in, R2 out
claw.bind_toggle(Controller.ButtonA);                         // each press flips it
lift.add_preset("down", 0).add_preset("high", 110);           // heights in degrees
lift.bind_preset(Controller.ButtonUp, "high");
lift.bind_preset(Controller.ButtonDown, "down");
```

For lift heights, try a number, download, and adjust until the lift stops
where you want.

Download and drive. **Keep the robot still when the program starts** until
the auton list appears: the inertial sensor is calibrating. If the robot goes
backward or spins when you push forward, a drive motor's `true`/`false` is
wrong in section 1.

## 4. Check the drive in auton

On the Brain, tap the right side of the screen until it shows **Drive Test**,
then run autonomous (use a competition switch, or the controller's
*Programs → your program → Timed Run*). The robot should drive forward 36
inches in three steps and come back to where it started.

- Goes too far or not far enough: check wheel diameter and gear ratio.
- Wobbles or overshoots: the PID numbers need tuning. They're in
  `default_constants()` in `src/autons/constants.cpp`. The tuner and graph in
  [GUIDE.md](GUIDE.md) let you change them from the controller and see the
  result on the Brain.

Then try **Turn Test**. The robot should end facing the way it started.

## 5. Write an auton

1. Make a new file in `src/autons/`, like `red_left.cpp`:
   ```cpp
   #include "vex.h"

   void red_left() {
     // Open the claw and raise the lift while driving up to a game object.
     claw.retract();
     lift.go_to("high");
     chassis.drive_distance(24);

     // Make sure the lift is up, then grab.
     lift.wait();
     claw.extend();
     wait(200, msec);   // give the piston a moment to close

     // Run the intake while backing up, then stop it.
     intake.spin();
     chassis.drive_distance(-12);
     intake.stop();

     chassis.turn_to_angle(90);
   }
   ```
2. Add `void red_left();` to the bottom of `include/autons.h`.
3. Add it to the list in `src/autons/auton_list.cpp`:
   ```cpp
   hh::selector::add("Red Left", "Grabs one, backs up, turns", red_left);
   ```

Each line runs after the one above it finishes, except `go_to`, which starts
the lift and moves straight on.

The most-used drive commands:

```cpp
chassis.drive_distance(24);    // forward 24 inches
chassis.drive_distance(-12);   // backward 12 inches
chassis.turn_to_angle(90);     // face 90 degrees. 0 is the way the robot faced
                               // at the start, and clockwise is positive.
chassis.left_swing_to_angle(45);   // turn using only the left side
```

All of JAR-Template's motions work, including `drive_to_point` and
`drive_to_pose` once position tracking is set up. See the
[JAR-Template docs](https://jacksonarearobotics.github.io/JAR-Template/), and
"Smarter autons" in [GUIDE.md](GUIDE.md) for doing two things at once, flowing
between motions and following curved paths.

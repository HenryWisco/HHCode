# Getting Started

This walks you from "the template is open in VS Code" to "the robot drives,
the mechanisms work, and an auton runs". If VS Code isn't set up yet, do
[SETUP.md](SETUP.md) first.

You will edit three files, in this order:

| File | What you do there |
|---|---|
| `src/robot-config.cpp` (and `include/robot-config.h`) | Describe your robot: ports, wheels, mechanisms |
| `src/main.cpp` | Choose buttons, lift heights, and drive style |
| `src/autons/` | Write your autons |

Leave the `JAR-Template` and `hh` folders alone. That's the library.

## 1. Describe your robot

Open `src/robot-config.cpp`. It has four numbered steps with instructions
in the comments. Everything marked `CHANGE ME` is an example value.

1. **Drive motors.** Set each motor's port, cartridge color and whether it's reversed.
2. **The chassis.** Set the inertial sensor port, wheel size and gear ratio.
   Leave the drive setup as `ZERO_TRACKER_NO_ODOM` for now.
3. **Mechanism motors and pistons.**
4. **Mechanisms.** Wrap them in blocks (see the table below).

Every time you add, remove or rename something in `robot-config.cpp`, make
the matching change in `include/robot-config.h`.

Building an X-drive, or using tracking wheels? Copy a ready-made chassis
block from [CONFIG-RECIPES.md](CONFIG-RECIPES.md).

**Build** (bottom bar) after each step. Fix the first error in the list
before looking at the others.

## 2. The mechanism blocks

| Block | Use it for | Commands |
|---|---|---|
| `hh::Spinner` | Roller intake, conveyor | `spin()`, `spin(8)`, `reverse()`, `stop()`, `spin_for(500)` |
| `hh::Positioner` | Lift, arm, wrist | `go_to("high")`, `go_to(45)`, `wait()`, `manual(10)`, `hold()` |
| `hh::Pneumatic` | Claw, clamp, wings | `extend()`, `retract()`, `toggle()` |

Making one takes two lines in `robot-config.cpp` and one in `robot-config.h`:

```cpp
// robot-config.cpp
motor IntakeMotor = motor(PORT7, ratio6_1, false);
hh::Spinner intake = hh::Spinner(motor_group(IntakeMotor));

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
- **Piston backward?** If `extend()` retracts it, add
  `claw.set_reversed(true);` at the top of `setup_presets()` in `main.cpp`.
- **Pistons must be listed below the chassis** in `robot-config.cpp`
  (they already are). Don't move Step 3 above Step 2.

## 3. Buttons, lift heights and drive style

Open `src/main.cpp`.

- **`setup_controls()`** has one line per button. Change the button names or
  the actions between the `{ }`.
- **`setup_presets()`** has the named lift heights in degrees. Try a number,
  download, and adjust until the lift stops where you want.
- **`usercontrol()`** picks the drive style: `control_arcade()`,
  `control_tank()` or `control_holonomic()` (X-drive and mecanum).

Download and drive. If the robot goes backward or spins when you push
forward, a drive motor's `reversed` is wrong in `robot-config.cpp`.

## 4. Check the drive in auton

On the Brain, tap the screen until it shows **Test: drive**, then run
autonomous (use a competition switch, or the controller's
*Programs → your program → Timed Run*). The robot should drive forward 36
inches in three steps and come back to where it started.

- Goes too far or not far enough: check wheel diameter and gear ratio.
- Wobbles or overshoots: the PID numbers need tuning. They're in
  `default_constants()` in `src/autons.cpp`. See the
  [JAR-Template tuning docs](https://jacksonarearobotics.github.io/JAR-Template/).

Then try **Test: turn**. The robot should end facing the way it started.

## 5. Write an auton

1. Copy `src/autons/example_auton.cpp` to a new file in the same folder,
   like `red_left.cpp`.
2. Rename the function inside it (`void red_left(){ ... }`) and change the steps.
3. Add `void red_left();` to `include/autons.h`.
4. Add `{"Red left", red_left},` to the auton list at the top of `src/main.cpp`.

The most-used drive commands:

```cpp
chassis.drive_distance(24);    // forward 24 inches
chassis.drive_distance(-12);   // backward 12 inches
chassis.turn_to_angle(90);     // face 90 degrees (clockwise from the start)
chassis.left_swing_to_angle(45);   // turn using only the left side
```

All of JAR-Template's motions work, including `drive_to_point` and
`drive_to_pose` once position tracking is set up. See the
[JAR-Template docs](https://jacksonarearobotics.github.io/JAR-Template/).

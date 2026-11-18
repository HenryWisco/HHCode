# HHCode Guide

How to use the HHCode template, from an empty robot to a tuned competition
program. Read "The big picture" once, then jump to the part you need.

- New computer? Do [SETUP.md](SETUP.md) first.
- First time with the template? [GETTING-STARTED.md](GETTING-STARTED.md) is a
  shorter walkthrough of sections 1 to 3 below.
- Ready-made chassis setups are in [CONFIG-RECIPES.md](CONFIG-RECIPES.md).

**Contents**

- [The big picture](#the-big-picture)
- [1. Get the robot driving](#1-get-the-robot-driving)
- [2. Add mechanisms](#2-add-mechanisms)
- [3. Buttons and one-button sequences](#3-buttons-and-one-button-sequences)
- [4. Autons](#4-autons)
- [5. Tuning](#5-tuning)
- [6. Match features: timer, endgame lock, driver profiles](#6-match-features)
- [7. Checking the robot](#7-checking-the-robot)
- [8. Match-day checklist](#8-match-day-checklist)
- [9. When something goes wrong](#9-when-something-goes-wrong)
- [10. Getting a new version of the template](#10-getting-a-new-version-of-the-template)
- [11. How the library is organized](#11-how-the-library-is-organized)

## The big picture

HHCode is built on three ideas.

1. **You describe your robot in one file.** `src/robot-config.cpp` holds the
   ports, the chassis, the mechanisms and the buttons. Nothing about your
   robot is hidden anywhere else.
2. **Mechanisms are built from blocks.** An intake is an `hh::Spinner`, a lift
   is an `hh::Positioner`, a piston is an `hh::Pneumatic`. One line makes
   one, and it then works in both autonomous and driver control.
3. **An auton is a list of commands**, run from top to bottom:
   `chassis.drive_distance(24);`, `lift.go_to("high");`, and so on.

### The files you edit

| You want to change... | Edit this file |
|---|---|
| Motor ports, the chassis, mechanisms, buttons | `src/robot-config.cpp` (and add an `extern` line in `include/robot-config.h`) |
| An auton routine | a file in `src/autons/` (declare it in `include/autons.h`) |
| Which autons show on the Brain | `src/autons/auton_list.cpp` |
| PID tuning numbers | `src/autons/constants.cpp` |

You normally don't touch `main.cpp`, and you never edit `src/hh/` or
`src/JAR-Template/`. Those two folders are the library, and they are replaced
when a new version of the template comes out.

### The `extern` rule

Everything you create in `robot-config.cpp` needs a matching line in
`include/robot-config.h`, so your autons can see it. It is the same line with
`extern` in front and without the part in brackets:

```cpp
// robot-config.cpp                              // robot-config.h
motor IntakeMotor = motor(PORT8, ratio6_1, false);   extern motor IntakeMotor;
hh::Spinner intake(IntakeMotor);                     extern hh::Spinner intake;
```

Forgetting it gives the build error `use of undeclared identifier`.

### What the examples mean by "section 1", "section 3" and `configure_robot()`

`robot-config.cpp` has four numbered sections, top to bottom:

| Section | What goes there |
|---|---|
| 1. Motors and sensors | One line per motor or sensor |
| 2. Chassis | The drivetrain |
| 3. Mechanisms | One line per block |
| 4. `configure_robot()` | Settings and buttons. This is a function, so these lines go inside its `{ }` |

Keep them in that order. Section 3 must stay below section 2 (see
[When something goes wrong](#9-when-something-goes-wrong)).

### What happens when the program runs

1. **Start-up.** The inertial sensor calibrates for about 2 seconds. **Keep
   the robot still** until the auton list appears on the Brain.
2. **Before the match.** The Brain shows the auton list. Tap to choose.
3. **Autonomous.** The chosen routine runs, top to bottom.
4. **Driver control.** Every 20 ms the program reads the joysticks and
   buttons and drives the robot.

A background task runs the whole time, 100 times a second. It is what moves a
lift to its target, clears an intake jam, sorts colors and watches for hot or
unplugged motors. That is why `lift.go_to("high");` in an auton doesn't wait:
the background task does the moving while your auton carries on.

## 1. Get the robot driving

In `src/robot-config.cpp`:

1. **Section 1:** create your drive motors. Each is (port, cartridge,
   reversed). Cartridges: `ratio36_1` is red (100 rpm), `ratio18_1` is green
   (200 rpm), `ratio6_1` is blue (600 rpm).
   ```cpp
   motor LeftFront = motor(PORT1, ratio6_1, true);
   motor LeftBack  = motor(PORT2, ratio6_1, true);
   motor RightFront = motor(PORT3, ratio6_1, false);
   motor RightBack  = motor(PORT4, ratio6_1, false);
   ```
2. **Section 2:** put them in the chassis: `motor_group(LeftFront, LeftBack)` for
   the left side, the same for the right, then the inertial sensor port, wheel
   diameter and gear ratio. Mecanum and X-drive robots also fill in the four
   wheel ports (LF, RF, LB, RB), with a minus sign on the same motors that
   are `true` (reversed) in section 1. Don't plug a reversed wheel motor into
   port 1: `-PORT1` can't be told apart from `PORT1`.
3. **Section 4:** pick `drive_type` and `style`.

**The short way.** Steps 1 and 2 can be one line instead. Replace the whole
`Drive chassis( ... );` block with:

```cpp
Drive chassis = hh::Chassis::tank({1, -2, 3}, {-4, 5, -6}, 10, 3.25, ratio6_1, 450);
```

Left ports, right ports (minus = reversed), inertial port, wheel diameter,
cartridge, wheel rpm. There are `mecanum` and `x_drive` versions and add-ons
for tracking wheels: see [CONFIG-RECIPES.md](CONFIG-RECIPES.md).

Build, download, and drive. Every drive motor must push the robot forward
when it spins forward. If the robot goes the wrong way or spins on the spot,
flip `true`/`false` (or the minus sign) on the motor that's backwards.

### Driver settings

All set in `configure_robot()`, each with `hh::driver::` in front:

| Setting | What it does |
|---|---|
| `drive_type` | `hh::TANK`, `hh::MECANUM` or `hh::X_DRIVE` |
| `style` | How the sticks drive (table below) |
| `throttle_curve`, `turn_curve` | 0 is a straight line. 2 to 5 gives finer control at low speed without losing top speed. |
| `turn_scale` | 0.8 limits turning to 80% speed. |
| `strafe_scale` | Same, for strafing on mecanum / X-drive. |
| `active_brake_kp` | 0 is off. Around 0.1 makes the robot resist being pushed when the sticks are released. |
| `stick_deadband` | Stick movement below this percent is ignored. Default 5. |
| `overheat_temp` | The controller rumbles and shows which mechanism or port is hot. Default 55 C; 0 turns it off. |
| `curve_tuning` | `true` lets the driver change curves while driving: Left/Right arrows for throttle, Y/A for turn. Saved to the SD card. Turn off for competition. |

| `style` | Left stick | Right stick |
|---|---|---|
| `hh::TANK_STICKS` | left side of the drive | right side of the drive |
| `hh::SPLIT_ARCADE` | forward / back | turn |
| `hh::LEFT_ARCADE` | forward / back and turn | nothing |
| `hh::RIGHT_ARCADE` | nothing | forward / back and turn |
| `hh::FLIPPED_ARCADE` | turn | forward / back |
| `hh::HOLONOMIC` (mecanum / X only) | move in any direction | turn |
| `hh::FIELD_CENTRIC` (mecanum / X only) | move in any direction on the field: "up" is always away from the driver | turn |

`FIELD_CENTRIC` needs an inertial sensor. To re-zero which way is "away from
the driver", bind a button:

```cpp
hh::on_press(Controller.ButtonY, [] { hh::driver::reset_field_centric(); });
```

If your team has more than one driver, see
[driver profiles](#driver-profiles).

## 2. Add mechanisms

Each mechanism is one line in section 3 of `robot-config.cpp`, one `extern`
line in `robot-config.h`, and its buttons in `configure_robot()`.

| Block | Use it for | Main commands |
|---|---|---|
| `hh::Spinner` | Intake, roller, conveyor | `spin()`, `reverse()`, `stop()` |
| `hh::Positioner` | Lift, arm, wrist | `go_to("high")`, `wait()`, `hold()` |
| `hh::Velocity` | Flywheel | `set_rpm(2400)`, `wait_until_ready()` |
| `hh::Pneumatic` | Clamp, claw, wings | `extend()`, `retract()`, `toggle()` |
| `hh::Pto` | Drive motors that shift to a hang or lift | `to_drive()`, `to_mechanism()` |
| `hh::ColorSorter` | Throwing out the other alliance's pieces | `keep(hh::RED)` |
| `hh::Sequence` | Several steps from one button | `start()`, `cancel()` ([section 3](#one-button-sequences)) |

> Don't name anything `clamp`. JAR-Template already uses that name. Use
> `mogo_clamp` or similar.

### Spinner: intakes, rollers, conveyors

```cpp
// section 1
motor IntakeMotor = motor(PORT8, ratio6_1, false);
// section 3
hh::Spinner intake(IntakeMotor);             // two motors: (IntakeLeft, IntakeRight)
// configure_robot()
intake.bind_hold(Controller.ButtonR1, Controller.ButtonR2);   // R1 in, R2 out
```

In an auton: `intake.spin();`, `intake.reverse();`, `intake.stop();`,
`intake.spin(60);` (60% speed).

Forward must be "in". If it runs backwards, flip `true`/`false` on the motor.

| Setting | What it does |
|---|---|
| `intake.speed = 80;` | The speed `spin()` and `reverse()` use, in percent. |
| `intake.jam_protection = true;` | Backs out by itself when something gets stuck, then carries on. |
| `intake.velocity_mode = true;` | Holds an exact speed under load instead of sending full power. |
| `intake.stop_mode = vex::hold;` | What the motors do when stopped: `coast`, `brake` or `hold`. |
| `intake.set_current_limit(1.5);` | Limits motor current (amps) to be gentle on game pieces. |

Use `bind_toggle` instead of `bind_hold` for press-on / press-off.

### Positioner: lifts, arms, anything that goes to an angle

```cpp
// section 3: motor, then driving gear teeth / driven gear teeth
hh::Positioner lift(LiftMotor, 12.0 / 60.0);
// configure_robot()
lift.add_preset("down", 0).add_preset("mid", 60).add_preset("high", 110);
lift.bind_preset(Controller.ButtonDown, "down");
lift.bind_preset(Controller.ButtonUp, "high");
lift.bind_manual(Controller.ButtonL1, Controller.ButtonL2);   // optional: move by hand
```

- **Positions are degrees of the mechanism**, and zero is wherever it sits
  when the program starts. Always start with it in the same place (all the
  way down).
- **It doesn't wait.** `lift.go_to("high");` starts the move and returns right
  away, so the lift rises while the robot drives. Add `lift.wait();` when the
  next step needs the lift to be there.
- To find a preset's number, try one, download, and adjust until the lift
  stops where you want.
- Positive voltage must make the position go up. If the lift runs the wrong
  way, flip `true`/`false` on the motor.

| Setting | What it does |
|---|---|
| `lift.kp`, `lift.ki`, `lift.kd` | The PID constants. Raise `kp` if it is slow or stops short; raise `kd` if it overshoots. |
| `lift.kg` | Constant volts to hold an elevator lift up against gravity. |
| `lift.kg_arm`, `lift.arm_level_position` | Volts to hold an arm level, and the position where it is level. |
| `lift.max_voltage` | Top power for moves, out of 12. |
| `lift.set_limits(0, 120);` | Stops it being driven past its travel. |
| `lift.use_rotation_sensor(LiftSensor);` | Measures with a rotation sensor on the mechanism instead of the motor. |
| `lift.zero();` | Calls the current position 0. |

### Velocity: flywheels

```cpp
hh::Velocity flywheel(FlywheelMotor, 3600);   // 3600 = top speed in rpm after gearing
flywheel.bind_toggle(Controller.ButtonX, 2400);
```

In an auton: `flywheel.set_rpm(2400); flywheel.wait_until_ready();`. Tuning:
`kf`, `kp`, `ki`, `ramp`, `ready_tolerance`.

### Pneumatic: pistons

```cpp
hh::Pneumatic mogo_clamp(Brain.ThreeWirePort.A);
mogo_clamp.bind_toggle(Controller.ButtonA);
```

In an auton: `mogo_clamp.extend();`, `.retract();`, `.toggle();`,
`.pulse(200);` (out, then back in after 200 ms). A second port fires two
solenoids together. `true` as the last argument starts it extended.
`mogo_clamp.reversed = true;` swaps extend and retract.

### Pto: drive motors that can switch to a mechanism

A PTO (power take-off) is a piston that shifts some drive motors over to a
hang or lift. Tank drives only.

```cpp
// Section 1: the PTO motors. Leave them OUT of the chassis's motor groups.
motor LeftPto = motor(PORT5, ratio6_1, true);
motor RightPto = motor(PORT6, ratio6_1, false);

// Section 3
hh::Pneumatic pto_piston(Brain.ThreeWirePort.D);
hh::Spinner hang(LeftPto, RightPto);
hh::Pto pto(LeftPto, RightPto);

// configure_robot()
pto.use_piston(pto_piston);          // piston extended = motors on the drive
pto.controls(hang);                  // hang only runs while the PTO is on it
pto.bind_toggle(Controller.ButtonX);
```

While the PTO is on the drive, its motors do whatever their side of the drive
does, in driver control and in every chassis motion, and the mechanism ignores
its buttons and commands. In an auton: `pto.to_mechanism();`, `pto.to_drive();`,
`pto.toggle();`, `pto.is_driving()`.

- Reverse the PTO motors the same way as the drive motors on their side:
  forward must drive the robot forward.
- `pto.use_piston(pto_piston, false);` if the piston works the other way round.
- `pto.start_on_drive = false;` if the robot starts with the PTO on the mechanism.
- `pto.shift_time = 100;` lets the motors go slack for 100 ms while the piston moves.
- Four PTO motors: `hh::Pto pto(LeftPtoA, LeftPtoB, RightPtoA, RightPtoB);`
- The mechanism can be any block. A `Positioner` on PTO motors loses track of
  its position while the motors are driving, so give it a rotation sensor on
  the mechanism itself (`use_rotation_sensor`). It forgets its target when
  the motors leave, so it never jumps when they come back.

### ColorSorter: throw out the other alliance's pieces

Needs an optical sensor that looks at the pieces as they go through the robot.

```cpp
// section 1
optical RingSensor = optical(PORT7);
// section 3
hh::ColorSorter sorter(RingSensor);
// configure_robot(): how to throw a piece out. Pick one.
sorter.eject_with(ejector);          // a piston (hh::Pneumatic) pops out
sorter.eject_with(intake, -100);     // or the intake runs backwards (0 = just stops)
```

Then say which color to keep, at the start of each auton or wherever you know
your alliance: `sorter.keep(hh::RED);` (blue gets thrown out), `hh::BLUE`, or
`hh::NO_COLOR` to turn sorting off. It stays set for driver control.
`sorter.bind_toggle(Controller.ButtonRight);` lets the driver pause it.

| Setting | What it does |
|---|---|
| `eject_delay` | ms from seeing the piece to throwing it out: how long the piece takes to travel from the sensor to the ejector. Default 0. |
| `eject_time` | ms the piston stays out or the intake runs backwards. Default 150. |
| `red_hue_min/max`, `blue_hue_min/max` | The hues that count as each color. Print `RingSensor.hue()` with a piece in front of the sensor if it gets them wrong. |
| `use_proximity` | `true` only trusts the color when something is close to the sensor. |
| `light_power` | Sensor light, 0 to 100. Keep it on; it makes colors far more reliable. |

`sorter.rejected()` counts the pieces thrown out, and `sorter.seen()` is what
the sensor sees right now.

### Sensors

Run something the moment a condition becomes true, in auton or driver control:

```cpp
hh::when([] { return IntakeSwitch.pressing(); }, [] { intake.stop(); });
```

The first `[] { ... }` is the question ("is the switch pressed?") and the
second is what to do when the answer changes from no to yes.

### Your own mechanism

When the blocks don't fit, make a class in `src/subsystems/` (header in
`include/subsystems/`):

```cpp
class Hang : public hh::Subsystem {
public:
  void update() override {
    // runs every 10 ms in auton and driver control
  }
  void on_disable() override {
    // runs when the robot is disabled between match periods; forget any
    // command here so the mechanism doesn't restart by itself
  }
};
```

Create one in section 3 like any block (`Hang hang;`) and the background task
runs it for you.

## 3. Buttons and one-button sequences

### Buttons

Each block has `bind_...` functions that cover most robots:

| Block | Bindings |
|---|---|
| Spinner | `bind_hold(in, out)`, `bind_hold(in)`, `bind_toggle(in, out)`, `bind_toggle(in)` |
| Positioner | `bind_preset(button, "name")`, `bind_manual(up, down)` |
| Velocity | `bind_toggle(button, rpm)` |
| Pneumatic | `bind_toggle(button)`, `bind_hold(button)` |
| Pto, ColorSorter | `bind_toggle(button)` |
| Sequence | `bind(button)`, `bind_hold(button)` |

For anything else, bind a button to any code you like. The code goes between
`[] {` and `}`:

```cpp
hh::on_press(Controller.ButtonB, [] { intake.spin(40); });
hh::while_held(Controller.ButtonX, [] { intake.reverse(); }, [] { intake.stop(); });
hh::toggle(Controller.ButtonY, [] { wing.extend(); }, [] { wing.retract(); });
```

- `on_press`: runs once each time the button goes down.
- `while_held`: the first part runs when it goes down, the second when it comes up.
- `toggle`: alternates between the two parts on each press.

**Shift key:** `hh::shifted(Controller.ButtonL2, Controller.ButtonR1)` means
"R1 while holding L2". It works anywhere a button does, which doubles the
number of buttons you have. Plain R1 bindings don't fire while L2 is held.

### One-button sequences

A sequence is several steps that run one after another from a single press.
You write the steps the same way you write an auton.

```cpp
// Section 3
hh::Sequence score_high([] {
  lift.go_to("high");
  lift.wait();              // wait for the lift to get there
  intake.reverse();
  wait(400, msec);
  intake.stop();
  lift.go_to("down");
});

// configure_robot()
score_high.bind(Controller.ButtonB);   // press to start, press again to cancel
```

The steps run in the background, so the driver keeps driving while they
happen.

| Setting or command | What it does |
|---|---|
| `score_high.bind(button);` | Press to start. Press again while it is running to cancel. |
| `score_high.bind_hold(button);` | Runs while the button is held. Letting go cancels it. |
| `score_high.on_cancel = [] { intake.stop(); };` | Runs when it is cancelled part way, to leave the robot in a safe state. |
| `score_high.uses_chassis = true;` | For steps that drive (`chassis.drive_distance(...)`). The sticks are ignored while it runs, and moving a stick cancels it, so the driver always gets control back. |
| `score_high.start();`, `.cancel();`, `.wait();`, `.is_running()` | For autons: start it in the background, and wait for it if you need to. |

- A cancel stops the steps wherever they are. Anything already started (a
  spinning intake) keeps going unless `on_cancel` stops it.
- Sequences are cancelled when the robot is disabled, so nothing carries over
  from autonomous into driver control.
- Up to 8 sequences.

## 4. Autons

### Writing one

1. Write a function in a new file in `src/autons/`, e.g. `red_left.cpp`:
   ```cpp
   #include "vex.h"

   void red_left() {
     intake.spin();
     chassis.drive_distance(24);
     lift.go_to("high");
     chassis.turn_to_angle(90);
   }
   ```
2. Declare it at the bottom of `include/autons.h`: `void red_left();`
3. Add it to `src/autons/auton_list.cpp`:
   ```cpp
   hh::selector::add("Red Left", "4 rings, touches ladder", red_left);
   ```

`src/autons/tests.cpp` has commented example routines to copy from.

### The chassis commands

Distances are in inches. Angles are in degrees: 0 is the way the robot faced
when the program started, and clockwise is positive (90 is to the right).

| Command | What it does |
|---|---|
| `chassis.drive_distance(24);` | Forward 24 inches. Negative is backward. |
| `chassis.drive_distance(24, 90);` | The same, while turning to face 90. |
| `chassis.turn_to_angle(90);` | Turns on the spot to face 90. |
| `chassis.left_swing_to_angle(45);` | Turns using only the left side (`right_swing_to_angle` for the right). |
| `chassis.strafe_distance(12);` | Mecanum / X-drive: 12 inches to the right. Negative is left. |
| `chassis.drive_stop(brake);` | Stops the drive. |
| `wait(300, msec);` | Does nothing for 0.3 seconds. |

With position tracking (see [CONFIG-RECIPES.md](CONFIG-RECIPES.md)) the robot
also knows where it is on the field as X and Y, and you can send it to
places:

| Command | What it does |
|---|---|
| `chassis.set_coordinates(0, 0, 0);` | "I am at X, Y, facing this heading." Put it first in the auton. |
| `chassis.turn_to_point(24, 24);` | Turns to face a point. |
| `chassis.drive_to_point(24, 24);` | Drives to a point. |
| `chassis.drive_to_pose(24, 24, 90);` | Drives to a point and arrives facing 90, in a curve. |
| `chassis.holonomic_drive_to_pose(24, 24, 90);` | Mecanum / X-drive: the same, moving and turning together. |
| `chassis.follow_path({{0, 24}, {24, 48}});` | Drives smoothly through a list of points. |

Call `odom_constants();` at the top of an auton that uses these. Every command
also has longer forms that set its speed and timeout for that one move; they
are listed in the
[JAR-Template docs](https://jacksonarearobotics.github.io/JAR-Template/).

### Picking an auton

Tap the left or right half of the Brain screen. When the robot is plugged
into a competition switch or the field and disabled, the controller's
Left/Right arrows work too and the controller shows the choice. With an SD
card in the Brain, the choice is remembered between runs.

To run an auton without a competition switch, use the controller:
*Programs → your program → Timed Run*.

To use the Brain screen for your own printing, set
`hh::selector::show_on_brain = false;` in `configure_robot()`.

### One auton for both sides of the field

Write and tune a routine once, then let the template flip it for the other
side:

```cpp
hh::selector::add_mirrored("Red Left", "Blue Right", "4 rings, touches ladder", red_left);
```

That adds two autons to the list. "Red Left" runs `red_left()` as written.
"Blue Right" runs the same function with every chassis motion flipped
left-to-right: `turn_to_angle(90)` turns left instead of right,
`drive_to_point(24, 36)` goes to (-24, 36), strafes, swings and paths swap
sides. Inside the routine, headings and coordinates still read as if it were
the original side, so nothing in your code changes.

Mechanisms are not flipped. If the routine uses something that only exists on
one side of the robot, check which version is running:

```cpp
if (hh::selector::mirrored()) right_wing.extend(); else left_wing.extend();
```

Mirroring only works if the field really is a mirror image on the other
side. Run the mirrored version on the real field before trusting it.

### Smarter autons

**Do two things at once.** `async()` makes the next motion return right away:

```cpp
chassis.async().drive_distance(36);
chassis.wait_until(12);    // 12 inches in...
intake.spin();             // ...start the intake
chassis.wait();            // then wait for the drive to finish
```

`wait_until` counts inches for drives and strafes, degrees for turns and swings.
`chassis.cancel_motion();` stops a background motion and brakes. Starting any
new motion also replaces one that is still running.
Mechanisms never need `async()`: `lift.go_to("high")` already runs in the
background.

**Don't stop between motions.** `chain()` ends the next motion as soon as it is
close, so the robot flows into the one after:

```cpp
chassis.chain().drive_distance(24);
chassis.chain().turn_to_angle(45);
chassis.drive_distance(12);          // the last one settles normally
```

**Pick which way to turn.** `turn_to_angle` takes the shorter way round. When
something is in the way, force it: `chassis.clockwise().turn_to_angle(270);`
or `chassis.counterclockwise().turn_to_angle(90);`. It applies to that one turn.

**Follow a curved path** (robots with position tracking). The robot drives
smoothly through the points instead of stopping at each one:

```cpp
chassis.follow_path({{0, 24}, {24, 48}, {48, 48}});
chassis.follow_path({{24, 24}, {0, 0}}, 10, true);   // 10 inch lookahead, backwards
```

**Strafe** (mecanum / X-drive, no position tracking needed):
`chassis.strafe_distance(12);` Positive is right. If it goes the wrong
distance, tune `chassis.strafe_multiplier`. Blue or red motors also need
`chassis.set_holonomic_cartridge(ratio6_1);` in `configure_robot()`.

### Wall resets: fixing position during a long auton

Position tracking drifts a little with every bump and wheel slip, and in a
long auton or a skills run it adds up. A distance sensor pointed at the field
wall measures where the robot really is, and `hh::wall_reset()` corrects X
or Y from it.

```cpp
// Section 1
distance FrontDistance = distance(PORT7);
distance LeftDistance = distance(PORT8);

// Section 3: sensor, which way it points, inches in FRONT of the robot's
// center (negative = behind), inches to the RIGHT of center (negative = left)
hh::WallSensor front_wall(FrontDistance, hh::FRONT, 6.5, 0);
hh::WallSensor left_wall(LeftDistance, hh::LEFT, 1.0, -5.5);

// In an auton, at a moment when the robot faces roughly square to the walls:
hh::wall_reset();
```

- The facing is `hh::FRONT`, `hh::RIGHT`, `hh::BACK` or `hh::LEFT`. The
  "center" is the point between the drive wheels.
- A sensor pointing at the top or bottom wall fixes Y; one pointing at a side
  wall fixes X. With one on the front and one on a side, a single
  `wall_reset()` fixes both.
- It needs position tracking and an inertial sensor. Heading is never
  changed.
- The walls are where the odometry map says they are: a 144 inch field with
  (0, 0) in the middle. If your autons put (0, 0) in a corner, set
  `hh::odom_map::center_x = 72; hh::odom_map::center_y = 72;`.

A reading is thrown away, and the position left alone, when it can't be
trusted. `hh::wall_reset()` returns how many of X and Y it fixed (0, 1 or 2),
and each sensor's `status` says why:

| `front_wall.status` | Meaning |
|---|---|
| `"ok"` | Used. |
| `"sees nothing"` | Nothing within range (`hh::walls::max_range`, default 70 inches). |
| `"not square to a wall"` | The sensor points more than `hh::walls::max_angle` (default 20 degrees) away from straight at a wall. |
| `"too far from expected"` | The answer is more than `hh::walls::max_correction` (default 8 inches) from where the robot thinks it is. Usually a game piece or robot was in the way. Set it to 0 to turn this check off. |

To check the sensors, open the [odometry map](#7-checking-the-robot): it
shows what each wall sensor reads (`Walls F23 L10`). `front_wall.reset();`
uses one sensor on its own, and `front_wall.angle_offset = 3;` corrects a
sensor that isn't mounted quite square.

## 5. Tuning

The numbers that decide how the robot drives in autonomous are in
`src/autons/constants.cpp`, with an explanation of each one in the comments.
Tune in this order: **Drive Test**, then **Turn Test**, then **Swing Test**.

### What the numbers do

| Number | Too low | Too high |
|---|---|---|
| kP (how hard it pushes toward the target) | Slow, stops short | Overshoots, shakes |
| kD (a brake that works harder the faster it approaches) | Overshoots | Jittery, slow to arrive |
| kI (a slow extra push when it sits just short) | Stops a little short | Wobbles around the target |

Start by raising kP until the robot just overshoots, then raise kD until it
stops overshooting. Leave kI at 0 unless it stops short every time.

### Tuning PID without re-downloading

Set `hh::tuner::enabled = true;` in `configure_robot()`. In driver control the
controller then works like this (your own button bindings are paused):

| Button | Does |
|---|---|
| Up / Down | Pick a constant (Drive kP, Turn kD, ...) |
| Left / Right | Lower / raise it |
| A | Run the auton chosen on the Brain screen |
| B | Stop it |
| X | Print all constants to the VS Code terminal, ready to paste into `constants.cpp` |

Pick "Drive Test" or "Turn Test" on the Brain, press A, watch, adjust, repeat.
The sticks still drive so you can reset the robot. Changes are lost when the
program restarts, so press X and copy them into `constants.cpp`. Autons that
call `default_constants()` or `odom_constants()` themselves will undo your
changes. **Turn the tuner off for competition.**

### Seeing what the PID is doing

Set `hh::graph::enabled = true;` in `configure_robot()` as well. Every chassis
motion is then drawn on the Brain screen as it runs:

- **Green line:** the error, the distance or angle still to go. It should drop
  to the grey zero line and stay there. Dipping below it and coming back is
  overshoot (lower kP or raise kD). Creeping in slowly is the opposite. A
  wiggle that doesn't die out means kD or kP is too high.
- **Orange line:** the voltage sent to the motors, from -12 to 12.
- **Top of the screen:** how long the motion took, the error it started with,
  how far it overshot, and the error it ended with.

The graph shows the most recent motion, so tune with an auton that does one
motion at a time. Tap the screen to get the auton list back. **Turn the graph
off for competition.**

### Other auton settings

All in `src/autons/constants.cpp`:

| Setting | What it does |
|---|---|
| `chassis.drive_slew`, `turn_slew` | Ramps the voltage up at the start of a motion (volts per second). Stops wheelies and wheel spin. Try 30 to 60. |
| `chassis.stall_time` | A motion gives up after the robot has been stuck this many ms, so one bad bump doesn't burn the whole auton. `chassis.interfered` is true afterwards. Try 300. |
| `chassis.drive_chain_error`, `turn_chain_error` | How close counts as "done" for `chain()`. |
| `chassis.path_lookahead` | Default lookahead for `follow_path`. Bigger is smoother, smaller is tighter. |
| `chassis.path_smoothing` | Rounds the corners of `follow_path` paths before driving them. 0 is off; try 0.8. |
| `chassis.set_drive_backward_constants(10, 1.5, 0, 10, 0);` | Separate max voltage, kP, kI, kD and startI for `drive_distance` with a negative distance, for robots that behave differently in reverse. |

## 6. Match features

### Match timer

The controller buzzes when driver control is nearly over: once with 30
seconds left and twice with 15. There is nothing to set up. The clock starts
by itself when driver control begins on a competition switch or a real field.

| Setting (in `configure_robot()`) | What it does |
|---|---|
| `hh::match_timer::driver_length = 60;` | How long driver control lasts, in seconds. 105 is a match; use 60 for driver skills. |
| `hh::match_timer::warn_at(10, "...");` | Adds a buzz with 10 seconds left. Dots are short buzzes, dashes are long, spaces are pauses. |
| `hh::match_timer::clear_warnings();` | Removes every warning, including the two built-in ones. |
| `hh::match_timer::show_on_controller = true;` | Shows the time left on the controller's second line. |

For practice without a competition switch, start the clock from a button:

```cpp
hh::on_press(Controller.ButtonUp, [] { hh::match_timer::start(); });
```

### Endgame lock

Keeps a mechanism from being fired by accident before the end of the match.
Wrap the button's action in `endgame_only`:

```cpp
hh::on_press(Controller.ButtonX, hh::match_timer::endgame_only([] { hang.extend(); }));
```

The button then only works in the last 30 seconds
(`hh::match_timer::endgame_time`). Pressed earlier, it gives one short buzz
and does nothing. When the clock isn't running (practice, the pits), the
button always works.

### Driver profiles

Each driver keeps their own stick layout and curves, and picks their name
before driving. In `configure_robot()`, after the `hh::driver` settings:

```cpp
hh::profiles::add("Alex");                           // the settings above, unchanged
hh::profiles::add("Sam", hh::TANK_STICKS, 3, 3);     // layout, throttle curve, turn curve
hh::profiles::add("Jo", [] {                         // or any driver settings at all
  hh::driver::style = hh::LEFT_ARCADE;
  hh::driver::turn_scale = 0.8;
});
```

- **To pick a driver**, tap **Driver: ... >** on the Brain's auton screen.
  While the robot is disabled on a field, the controller's Up and Down arrows
  work too, and the controller shows the name.
- The choice is saved to the SD card. The first profile added is the default.
- Every profile starts from the settings in `configure_robot()` and changes
  only what it says.
- With `curve_tuning` on, the curves a driver adjusts while driving are saved
  under their own name.
- Profiles change how the sticks drive. Button bindings are the same for
  every driver.

## 7. Checking the robot

**Ports page.** Tap **Ports>** in the blue bar at the top of the auton screen. It shows all
21 smart ports: what is plugged into each one and how hot each motor is.
Green is fine, orange is a hot motor, red is something that was there when the
program started and has stopped answering. Tap anywhere to go back. Use it
before every match: a motor that isn't on the page isn't plugged in.

**Unplugged warning.** If a cable comes out or a motor dies while the program
is running, the controller rumbles and shows `PORT 5 UNPLUGGED`. Turn it off
with `hh::ports::warn_unplugged = false;`.

**Hot motor warning.** When any motor reaches `hh::driver::overheat_temp`,
the controller rumbles and shows `HOT: Intake 56C` (or `HOT: Port 2 61C` for
a drive motor).

**No inertial sensor.** The bar shows `NO IMU!` when the inertial sensor isn't
found on the port you gave the chassis. Autons can't turn accurately without it.

**Odometry map.** Tap **Map>** in the blue bar. It shows a top-down picture of
the field with the robot as a yellow arrow, a blue trail of where it has been,
and its X, Y and heading as numbers. Tap anywhere to go back. It needs a
chassis setup with position tracking (anything except `ZERO_TRACKER_NO_ODOM`).

- **Finding coordinates for an auton:** open the map, push the robot by hand
  to where you want it to go, and read X and Y off the screen. Tracking starts
  at (0, 0) facing 0 when the program starts.
- **Watching an auton:** if the map is open when autonomous starts, it stays
  open and draws the path the robot takes.
- **Wall sensors:** if the robot has any, the bottom line shows what each one
  reads, e.g. `Walls F23 L10` (front 23 inches, left 10). `--` means it sees
  nothing.
- The map shows X and Y from -72 to 72 (a full field with (0, 0) in the
  middle). If your autons start at (0, 0) in a corner, add
  `hh::odom_map::center_x = 72; hh::odom_map::center_y = 72;` to
  `configure_robot()`. `hh::odom_map::field_size` changes how much it shows.

## 8. Match-day checklist

Before the event:

1. `hh::tuner::enabled`, `hh::graph::enabled` and `hh::driver::curve_tuning`
   are all `false`.
2. An SD card is in the Brain (it remembers the auton and the driver).
3. The program is downloaded, and a backup copy of the team folder exists.

Before each match:

1. Battery charged, air tank full.
2. Start the program and **keep the robot still** until the auton list shows.
3. Tap **Ports>**: every motor is green. Tap to go back.
4. The blue bar does not say `NO IMU!`.
5. The right **auton** is showing, and the right **driver**.
6. Mechanisms are in their starting positions (lift down).

## 9. When something goes wrong

| Problem | Cause and fix |
|---|---|
| Build error `use of undeclared identifier` | The `extern` line is missing from `include/robot-config.h`, or a new auton isn't declared in `include/autons.h`. |
| Many build errors at once | Fix the first one in the list. The rest are often caused by it. |
| A strange build error on a line like `hh::Spinner intake(motor_group(A, B));` | Give the block the motors directly: `hh::Spinner intake(A, B);` |
| Build error mentioning `clamp` | Something of yours is named `clamp`. Rename it (`mogo_clamp`). |
| Robot drives backward or spins when pushed forward | A drive motor's `true`/`false` (or minus sign) is wrong. |
| A piston doesn't work | Section 3 was moved above section 2. Mechanisms must be created after the chassis. Also check `reversed`, the 3-wire port letter, and the air. |
| Auton turns are wrong or the robot spins | The inertial sensor isn't on the port the chassis was given (`NO IMU!` on the Brain), or the robot was moved while it calibrated. |
| Drives too far or not far enough | Wheel diameter or gear ratio is wrong in the chassis. |
| Overshoots, wobbles, or creeps in slowly | PID tuning: [section 5](#5-tuning). |
| Lift goes to the wrong height | It wasn't all the way down when the program started. Zero is wherever it starts. |
| Lift runs away in one direction | Positive voltage must raise the position. Flip `true`/`false` on its motor. |
| A motion never finishes | It can't reach its target. It gives up at its timeout (see `constants.cpp`). Set `chassis.stall_time = 300;` to give up sooner when stuck. |
| Buttons do nothing, controller shows a constant's name | The PID tuner is on. Set `hh::tuner::enabled = false;`. |
| The Brain shows a graph instead of the auton list | The PID graph is on. Tap the screen, or set `hh::graph::enabled = false;`. |
| Position numbers don't change | The chassis setup is `ZERO_TRACKER_NO_ODOM`. Pick one with position tracking. |
| `wall_reset()` returns 0 | Read each sensor's `status` ([wall resets](#wall-resets-fixing-position-during-a-long-auton)) and check the readings on the map. |
| A mirrored auton goes wrong on one side | The field or the robot isn't a true mirror image. Test both versions on a real field. |

## 10. Getting a new version of the template

When your coach gives you a newer template, this replaces the library and
keeps your robot's files (`robot-config`, your autons, your PID constants,
`main.cpp`, `subsystems/`).

1. Unzip the new template somewhere, like `Downloads/HHCode-new`.
2. Open **your team's** folder in VS Code, then **Terminal → New Terminal**.
3. Windows: `.\tools\update-library.ps1 -From "C:\Users\you\Downloads\HHCode-new"`
   Mac: `sh tools/update-library.sh ~/Downloads/HHCode-new`
4. Click **Build**.

Your old library is saved in a `_library-backup-...` folder first. Delete that
folder once everything works. If Windows refuses to run the script, do it by
hand: delete `include/hh`, `src/hh`, `include/JAR-Template` and
`src/JAR-Template` from your folder and copy the same four folders in from
the new template.

## 11. How the library is organized

For anyone who wants to read or fix the library itself. Every file starts
with a comment saying what it does, and each `.h` file in `include/hh/` has
the instructions for that feature.

| File (`include/hh/*.h`, `src/hh/*.cpp`) | What it is |
|---|---|
| `subsystem` | The list of mechanisms, the background task that runs them, `hh::init()`, `hh::when()` |
| `buttons` | Button bindings and shift combos |
| `driver` | Joysticks to motors: stick layouts, curves, active brake, overheat warning |
| `spinner`, `positioner`, `velocity`, `pneumatic` | The four basic blocks |
| `color_sorter`, `pto`, `sequence` | The blocks built on top of them |
| `chassis` | The one-line chassis setup |
| `selector` | The auton list on the Brain and controller |
| `ports`, `odom_map`, `graph` | The Brain pages: ports, map, PID graph |
| `tuner` | The live PID tuner |
| `match_timer`, `profiles` | Match timer and endgame lock; driver profiles |
| `wall_reset` | Distance-sensor position correction |

`src/JAR-Template/` is JAR-Template v1.2.0. The chassis motions live in
`drive.cpp`; everything HHCode added or changed there is marked with a
`HHCode:` comment.

`tests/` holds a PC simulator that runs this same code against a pretend
robot (see `tests/README.md`). It checks the logic. It can't check anything
about real hardware, so a feature isn't proven until it has run on a robot.

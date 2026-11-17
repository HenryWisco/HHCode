# HHCode Guide

How to set up your robot and use the HHCode features. Do [SETUP.md](SETUP.md)
first. For autonomous driving functions (`drive_distance`, `turn_to_angle`,
odometry) see the [JAR-Template docs](https://jacksonarearobotics.github.io/JAR-Template/).

## Where things go

| You want to change... | Edit this file |
|---|---|
| Motor ports, the chassis, mechanisms, buttons | `src/robot-config.cpp` (and add an `extern` line in `include/robot-config.h`) |
| PID constants | `src/autons/constants.cpp` |
| Which autons show on the Brain | `src/autons/auton_list.cpp` |
| An auton routine | a file in `src/autons/` (declare it in `include/autons.h`) |

You normally don't touch `main.cpp`, `src/hh/` or `src/JAR-Template/`.

## 1. Get the robot driving

In `src/robot-config.cpp`:

1. **Section 1:** create your drive motors.
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

Build, download, and drive. If the robot goes the wrong way, flip `true`/`false`
on the motor that's backwards.

**When the program starts, keep the robot still** until the auton selector
appears. The inertial sensor is calibrating.

### Driver settings

All set in `configure_robot()`:

| Setting | What it does |
|---|---|
| `hh::driver::drive_type` | `hh::TANK`, `hh::MECANUM` or `hh::X_DRIVE` |
| `hh::driver::style` | `TANK_STICKS`, `SPLIT_ARCADE`, `LEFT_ARCADE`, `RIGHT_ARCADE`, `FLIPPED_ARCADE`, `HOLONOMIC`, `FIELD_CENTRIC` (all with `hh::` in front) |
| `throttle_curve`, `turn_curve` | 0 is a straight line. 2 to 5 gives finer control at low speed without losing top speed. |
| `turn_scale` | 0.8 limits turning to 80% speed. |
| `strafe_scale` | Same, for strafing on mecanum / X-drive. |
| `active_brake_kp` | 0 is off. Around 0.1 makes the robot resist being pushed when the sticks are released. |
| `stick_deadband` | Stick movement below this percent is ignored. Default 5. |
| `overheat_temp` | The controller rumbles and shows which mechanism is hot. Default 55 C; 0 turns it off. |
| `curve_tuning` | `true` lets the driver change curves while driving: Left/Right arrows for throttle, Y/A for turn. Saved to the SD card. Turn off for competition. |

`FIELD_CENTRIC` needs an inertial sensor. To re-zero which way is "away from
the driver", bind a button:

```cpp
hh::on_press(Controller.ButtonY, [] { hh::driver::reset_field_centric(); });
```

## 2. Add mechanisms

Each mechanism is one line in section 3 of `robot-config.cpp`, one `extern`
line in `robot-config.h`, and its buttons in `configure_robot()`.

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

Options: `intake.speed = 80;`, `intake.jam_protection = true;` (backs out by
itself when stuck), `intake.velocity_mode = true;`, `intake.stop_mode = vex::hold;`,
`intake.set_current_limit(1.5);`. Use `bind_toggle` instead of `bind_hold` for
press-on / press-off.

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

Positions are degrees of the mechanism, and zero is wherever it sits when the
program starts. In an auton, `lift.go_to("high");` starts the move and returns
right away, so the lift rises while the robot drives. Add `lift.wait();` to
pause until it arrives.

Tuning: `lift.kp`, `lift.ki`, `lift.kd`; `lift.kg` (volts to hold an elevator
up) or `lift.kg_arm` with `lift.arm_level_position` (volts to hold an arm
level); `lift.max_voltage`; `lift.set_limits(0, 120);` to stop it being driven
past its travel; `lift.use_rotation_sensor(LiftSensor);`.

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

Tuning:

| Setting | What it does |
|---|---|
| `eject_delay` | ms from seeing the piece to throwing it out: how long the piece takes to travel from the sensor to the ejector. Default 0. |
| `eject_time` | ms the piston stays out or the intake runs backwards. Default 150. |
| `red_hue_min/max`, `blue_hue_min/max` | The hues that count as each color. Print `RingSensor.hue()` with a piece in front of the sensor if it gets them wrong. |
| `use_proximity` | `true` only trusts the color when something is close to the sensor. |
| `light_power` | Sensor light, 0 to 100. Keep it on; it makes colors far more reliable. |

`sorter.rejected()` counts the pieces thrown out, and `sorter.seen()` is what
the sensor sees right now.

### Buttons

The `bind_...` functions cover most robots. For anything else:

```cpp
hh::on_press(Controller.ButtonB, [] { intake.spin(40); });
hh::while_held(Controller.ButtonX, [] { intake.reverse(); }, [] { intake.stop(); });
hh::toggle(Controller.ButtonY, [] { wing.extend(); }, [] { wing.retract(); });
```

**Shift key:** `hh::shifted(Controller.ButtonL2, Controller.ButtonR1)` means
"R1 while holding L2". It works anywhere a button does. Plain R1 bindings don't
fire while L2 is held.

### Sensors

Run something the moment a condition becomes true, in auton or driver control:

```cpp
hh::when([] { return IntakeSwitch.pressing(); }, [] { intake.stop(); });
```

### Your own mechanism

When the blocks don't fit, make a class in `src/subsystems/`:

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

## 3. Autons

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

**Follow a curved path** (odom robots). The robot drives smoothly through the
points instead of stopping at each one:

```cpp
chassis.follow_path({{0, 24}, {24, 48}, {48, 48}});
chassis.follow_path({{24, 24}, {0, 0}}, 10, true);   // 10 inch lookahead, backwards
```

**Strafe** (mecanum / X-drive, no odom needed): `chassis.strafe_distance(12);`
Positive is right. If it goes the wrong distance, tune
`chassis.strafe_multiplier`. Blue or red motors also need
`chassis.set_holonomic_cartridge(ratio6_1);` in `configure_robot()`.

**Settings** in `src/autons/constants.cpp`:

| Setting | What it does |
|---|---|
| `chassis.drive_slew`, `turn_slew` | Ramps the voltage up at the start of a motion (volts per second). Stops wheelies and wheel spin. Try 30 to 60. |
| `chassis.stall_time` | A motion gives up after the robot has been stuck this many ms, so one bad bump doesn't burn the whole auton. `chassis.interfered` is true afterwards. Try 300. |
| `chassis.drive_chain_error`, `turn_chain_error` | How close counts as "done" for `chain()`. |
| `chassis.path_lookahead` | Default lookahead for `follow_path`. Bigger is smoother, smaller is tighter. |

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

**Picking an auton:** tap the left or right half of the Brain screen. When the
robot is plugged into a competition switch or the field and disabled, the
controller's Left/Right arrows work too and the controller shows the choice.
With an SD card in the Brain, the choice is remembered between runs.

To use the Brain screen for your own printing, set
`hh::selector::show_on_brain = false;` in `configure_robot()`.

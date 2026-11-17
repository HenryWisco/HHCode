# Roadmap

JAR-Template runs on the VEX API (`vex.h`). EZ-Template runs on PROS
(`pros::` API), so its code can't be copied in. Its features get rebuilt here
on top of JAR's classes. Nothing below is written yet; this list is the plan to
agree on before coding.

## Design goals (from Henry)

**Bridge supreme usability and high-end competitiveness.** A rookie should get a
driving, auton-capable robot by filling in ports and numbers. A top team should be
able to tune or replace every piece without fighting the template.

1. **Any drivetrain:** tank, mecanum, or X-drive, picked with one setting in
   `robot-config.cpp`. The same auton calls (`drive_distance`, `turn_to_angle`,
   `drive_to_pose`, ...) work on all three. Holonomic drives also get `strafe` and
   field-centric driver control.
2. **Any mechanism:** teams build subsystems from a small set of reusable
   blocks instead of writing motor code from scratch, and can write fully custom
   ones when the blocks don't fit.
3. **Two layers for everything:**
   - *Easy:* one-line setup with defaults that work on a typical robot.
   - *Pro:* every constant, exit condition and loop is exposed. Teams can
     override any of them per call or replace the whole thing.
4. **One place to configure:** ports, gear ratios, wheel sizes and button
   mappings all live in `robot-config.cpp`. Nothing team-specific is buried in
   library code.

### Drivetrain: what JAR already does vs. what's missing

| | Tank | Mecanum | X-drive |
|---|---|---|---|
| Driver control | `control_tank`, `control_arcade` ✅ | `control_holonomic` ✅ | `control_holonomic` ✅ |
| PID drive/turn/swing | ✅ | ✅ | ✅ |
| Odometry | No trackers, 1 or 2 trackers ✅ | **Only with 2 tracking wheels** | **Only with 2 tracking wheels** |
| Drive-to-pose with strafing | n/a | ✅ (needs odom) | ✅ (needs odom) |
| Strafe a distance (no odom) | n/a | ❌ add | ❌ add |
| Field-centric driver control | n/a | ❌ add | ❌ add |
| Mecanum vs X tuning differences | n/a | ❌ add (strafe multiplier) | ❌ add |

### Subsystem building blocks (`hh/`)

| Block | Covers | Easy layer | Pro layer |
|---|---|---|---|
| `hh::Spinner` | intakes, rollers, conveyors | `intake.spin()`, `.reverse()`, `.stop()` | jam detection + auto-reverse, voltage/velocity mode, current limits |
| `hh::Positioner` | lifts, arms, wall-stake mechs, any "go to angle" | named presets: `lift.go_to("high")` | PID + feedforward constants, soft limits, hold power, motion exit conditions |
| `hh::Velocity` | flywheels, any "spin at speed" | `flywheel.set_rpm(2400)`, `.ready()` | velocity PID + feedforward, ramp, ready tolerance |
| `hh::Pneumatic` | clamps, wings, doinkers, any piston | `clamp.toggle()`, `.extend()`, `.retract()` | multiple solenoids, timed pulse, start state |
| `hh::Sensor` hooks | limit switch, distance, optical, rotation | `when(limit.pressed(), intake.stop)` | color sorting, auto-stop on detect |
| `hh::Subsystem` base | anything else a team invents | subclass + write `update()` | runs in the shared background task with the others |
| Button bindings | driver control | `bind(R1, intake.spin)` style, one line per button | hold vs toggle vs press, shift-key combos |

Every block works in both autonomous and driver control, and can run in the
background during a drive (e.g. lift to "high" while driving to a goal).

## What JAR-Template already gives us (v1.2.0)

- Generic PID class with settle/timeout exit conditions
- Odometry (with or without tracking wheels)
- Tank and holonomic (X/mecanum) drive support
- `drive_distance`, `turn_to_angle`, swing turns, `drive_to_point`, `drive_to_pose` (boomerang), holonomic variants
- Basic auton selector

## EZ-Template features to add

| Feature | EZ-Template | Notes for our version |
|---|---|---|
| Auton selector on the Brain screen with names + descriptions, saved to SD card | `ez::as::auton_selector` | Replace JAR's basic selector. Also allow picking from the controller. |
| Live PID tuner on the controller | `pid_tuner_*` | Lets students tune without re-downloading. |
| Driver joystick curves, adjustable live | `opcontrol_curve_*` | Tank, single-stick and split arcade. |
| Active brake in driver control | `opcontrol_drive_activebrake_set` | |
| Slew (ramp up speed at the start of a motion) | `slew_drive_set` | Stops wheelies/tipping. |
| Async motions with `wait_until(distance)` | `pid_wait_until` | Start an intake partway through a drive. |
| Motion chaining | `pid_wait_quick_chain` | |
| Pure pursuit path following | `pid_odom_set({...})` | JAR has point/pose, not paths. |
| Piston helper class | `ez::Piston` | Toggle on a button with one line. |
| Tug-of-war / interference detection | `interfered` | Exits a motion if the robot is pushed. |
| Overheat warning | | Rumble the controller when motors get hot. |

## Henry's custom features

_To be filled in as Henry outlines them._

## Prior art worth reading before we write each feature

Two newer MIT-licensed templates already built EZ-style features on the VEX
extension. We can learn from (or, with credit, reuse parts of) them:

- **RW-Template** (https://github.com/richardbwang/RW-Template): auton selector on Brain and
  controller, driver curves, async motions with `waitUntilTraveled()`, pure pursuit/RAMSETE,
  and a simulator. Keeps all team code in a `custom/` folder so the library updates cleanly.
- **mikLib** (https://github.com/EthanMik/mikLib): JAR-style motions plus realtime PID
  graphing/tuning UI, controller + Brain auton selector, SD-card saving.

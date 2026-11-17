# Roadmap

JAR-Template runs on the VEX API (`vex.h`). EZ-Template runs on PROS
(`pros::` API), so its code can't be copied in. Its features get rebuilt here
on top of JAR's classes. Nothing below is written yet; this list is the plan to
agree on before coding.

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

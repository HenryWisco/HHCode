# HHCode Implementation Spec (repo copy)

The full spec, with discussion, is a Claude Doc in Henry's HHCode project:
https://claude.ai/artifact/4y4jnAV3dQgnyjUAPrEJSq. This file is the working
summary that lives with the code. Keep the status column current.

## Goal

A VEX V5 C++ template that combines JAR-Template's readable core with
EZ-Template's competition features. A rookie team should have a driving,
auton-capable robot in one meeting. A top team should be able to tune or
replace any part.

**Non-goals for v1:** PROS support, Python, copying EZ-Template code.

## Integration map

**Keep** = JAR code used as is. **Port** = EZ behavior rebuilt on the VEX API.
**New** = in neither template.

Status key: **Sim** = written, builds for the Brain and passes the PC simulator
tests in `tests/`, but has not been run on a robot. Nothing in `hh/` has been
run on a robot yet.

| Capability | Source | Where | Status |
|---|---|---|---|
| PID with settle and timeout exits | Keep | JAR `PID` | Done (JAR) |
| Odometry, 0/1/2 trackers + IMU (tank) | Keep | JAR `Odom` | Done (JAR) |
| Drive, turn, swing, drive-to-point, boomerang drive-to-pose | Keep | JAR `Drive` | Done (JAR) |
| Holonomic drive-to-pose | Keep | JAR | Done (JAR) |
| One commented config file | New | `src/robot-config.cpp` | Sim |
| `hh::Spinner`, `hh::Positioner`, `hh::Pneumatic` | New / EZ `Piston` | `hh/` | Sim |
| Button bindings (hold, press, release, toggle) and shift button | New | `hh/buttons` | Sim |
| Brain + controller selector, descriptions, SD save | Port | `hh/selector` | Sim |
| Async motions + `wait_until` | Port | `chassis.async()` | Sim |
| Motion chaining | Port | `chassis.chain()` | Sim |
| Slew | Port | `chassis.drive_slew`, `turn_slew` | Sim |
| Joystick curves, deadband, turn scale, active brake | Port | `hh/driver` | Sim |
| One setting per drive type (tank / mecanum / X) | New | `hh::driver::drive_type` | Sim |
| Field-centric holonomic driving | New | `hh::FIELD_CENTRIC` | Sim |
| Background scheduler, `hh::Subsystem` base class | New | `hh/subsystem` | Sim |
| `hh::Velocity` (flywheels) | New | `hh/velocity` | Sim |
| Sensor triggers (`hh::when(...)`) | New | `hh/subsystem` | Sim |
| Color sorting | New | `hh::ColorSorter` | Sim |
| Pro layer for blocks: jam detect, PID + feedforward, soft limits | New | block settings | Sim |
| Live PID tuner on the controller | Port | `hh/tuner` | Sim |
| PID graph on the Brain | New | `hh/graph` | Sim |
| Interference (stall) detection | Port | `chassis.stall_time` | Sim |
| Pure pursuit | Port | `chassis.follow_path` | Sim (no path smoothing) |
| Holonomic odom without trackers, strafe a distance | New | `HOLONOMIC_ZERO_TRACKER_ODOM`, `strafe_distance` | Sim |
| Overheat rumble | New | `hh/driver` | Sim |
| PC simulator and tests | New | `tests/` | Done |
| Red/blue mirrored autons | New | `hh::selector::add_mirrored` | Sim |
| `hh::Chassis` wrapper: signed ports, one call per drive type | New | | Not started |
| Separate forward/backward constants | Port | `chassis.set_drive_backward_constants` | Sim |
| Turn direction (shortest/clockwise/counterclockwise) | Port | `chassis.clockwise()` | Sim (IMU scaling is JAR's gyro scale) |
| Path smoothing for pure pursuit | Port | `chassis.path_smoothing` | Sim |
| Brain ports page: motor temps, unplugged-port warning | New | `hh/ports` | Sim |
| Brain odometry map | New | | Not started |
| PTO | Port | | Not started |
| Update script (replace library folders, keep team files) | New | `tools/update-library` | Done (tested on copies of this repo) |
| CI: run the simulator tests on GitHub | New | `.github/workflows` | Added |
| Henry's own custom features | New | | Not listed yet |

## How the built API differs from the original plan

The first draft of this spec planned a new `hh::Chassis` class and an options
struct (`chassis.drive(48, hh::Opts().async())`). What was built instead keeps
JAR's `Drive chassis` and adds to it, so every JAR function and the JAR docs
still apply:

```cpp
chassis.async().drive_distance(48);   // returns right away
chassis.wait_until(30);
intake.spin();
chassis.wait();

chassis.chain().drive_distance(24);   // flows into the next motion

hh::selector::add("Red Left", "Rush mid goal, score 4", red_left);
hh::selector::add_mirrored("Red Left", "Blue Right", "Same routine, both sides", red_left);
```

Still planned in the original shape:

```cpp
// chassis in one call, negative port = reversed
hh::Chassis chassis = hh::Chassis::tank({1, -2, 3}, {-4, 5, -6}, /*imu*/10, 3.25, 450);
```

## Still open

- Henry's own custom features (not listed yet).
- Nothing has been run on a robot. See the on-robot check in `CLAUDE.md`.
- Whether the VEX SDK can be downloaded in GitHub Actions for a CI build of the
  robot program (the simulator tests don't need it).

# HHCode Implementation Spec (repo copy)

The full spec, with discussion, is a Claude Doc in Henry's HHCode project:
https://claude.ai/artifact/4y4jnAV3dQgnyjUAPrEJSq. This file is the working
summary that lives with the code. Keep the status column current.

## Goal

A VEX V5 C++ template that combines JAR-Template's readable core with
EZ-Template's competition features. A rookie team should have a driving,
auton-capable robot in one meeting. A top team should be able to tune or
replace any part.

**Non-goals for v1:** PROS support, Python, copying EZ-Template code, a simulator.

## Integration map

**Keep** = JAR code used as is. **Port** = EZ behavior rebuilt on the VEX API.
**New** = in neither template.

| Capability | Source | Reference | Status |
|---|---|---|---|
| PID with settle and timeout exits | Keep | JAR `PID` | Done (JAR) |
| Odometry, 0/1/2 trackers + IMU (tank) | Keep | JAR `Odom` | Done (JAR) |
| Drive, turn, swing, drive-to-point, boomerang drive-to-pose | Keep | JAR `Drive` | Done (JAR) |
| Holonomic drive-to-pose (needs 2 trackers) | Keep | JAR | Done (JAR) |
| One commented config file | New | | Done v0.1 |
| `hh::Spinner`, `hh::Positioner`, `hh::Pneumatic` easy layer | New / EZ `Piston` | | Done v0.1 |
| Button bindings (hold, press, toggle) | New | | Done v0.1 |
| Named auton list on the Brain | Port | EZ `auton_selector` | Basic in v0.1 |
| Brain + controller selector, descriptions, SD save | Port | EZ `auton_selector`, `sdcard` | M2 |
| Red/blue mirrored autons | New | | M2 |
| Async motions + `wait_until(distance)` | Port | EZ `pid_tasks.cpp` | M2 |
| Motion chaining | Port | EZ `pid_wait_quick_chain` | M2 |
| Separate forward/backward constants, slew | Port | EZ `slew.cpp` | M2 |
| Joystick curves, deadband, max speed, active brake | Port | EZ `user_input.cpp` | M1 |
| `hh::Chassis` wrapper: signed ports, one call per drive type | New | | M1 |
| Field-centric holonomic driving | New | | M1 |
| Background scheduler, `hh::Subsystem` base class | New | | M3 |
| `hh::Velocity` (flywheels) | New | | M3 |
| Sensor triggers (`hh::when(...)`), color sorting | New | | M3 |
| Pro layer for blocks: jam detect, PID + feedforward, soft limits | New | | M3 |
| Shift button for bindings | New | | M3 |
| Live PID tuner on the controller | Port | EZ `pid_tuner.cpp` | M4 |
| Advanced exit conditions, interference detection | Port | EZ `exit_conditions.cpp` | M4 |
| Pure pursuit with path smoothing | Port | EZ `purepursuit_math.cpp` | M4 |
| Holonomic odom without trackers, `strafe(in)` | New | | M4 |
| Turn behavior (shortest/left/right), IMU scaler | Port | EZ | M4 |
| Brain screens: odom map, motor temps, unplugged ports | New | | M4 |
| Overheat rumble, PTO | New / Port | EZ `pto.cpp` | M4 |
| Update script (replace library folders, keep team files) | New | RW-Template | M5 |
| CI compile check, per-block docs, rookie test | New | | M5 |

## Planned API shapes (not built yet)

```cpp
// M1: chassis in one call, negative port = reversed
hh::Chassis chassis = hh::Chassis::tank({1, -2, 3}, {-4, 5, -6}, /*imu*/10, 3.25, 450);
hh::Chassis chassis = hh::Chassis::xdrive(/*FL*/1, /*FR*/-2, /*BL*/3, /*BR*/-4, 10, 3.25, 450);

// M2: async motion, then act partway through
chassis.drive(48, hh::Opts().async());
chassis.wait_until(30);
intake.spin();
chassis.wait();

// M2: selector
hh::autons.add("Red left", "Rush mid goal, score 4", red_left);
hh::autons.add_mirrored("Left side", "Same routine, both alliances", left_side);
```

Settings structs use chained setters (`hh::Opts().max_v(8).timeout(1500)`)
unless designated initializers are confirmed to build with the VEX toolchain.

## Milestones

Each ends with something a team can download and drive.

1. **v0.1 (done, needs on-robot test):** config file, three blocks, bindings, named auton list, docs.
2. **M1 Foundation:** `hh::Chassis` for tank/X/mecanum, driver modes and curves, field-centric.
3. **M2 Autons:** Brain + controller selector with SD save and mirroring, async, chaining, slew.
4. **M3 Mechanisms:** scheduler, `Subsystem`, `Velocity`, sensor triggers, Pro layer.
5. **M4 Competitive:** PID tuner, exits, interference, pure pursuit, holonomic odom, Brain screens.
6. **M5 Release:** update script, CI, docs per block, rookie test, `v1.0.0`.

## Still open

- Henry's own custom features (not listed yet).
- Whether the VEX SDK can be downloaded in GitHub Actions for a CI build.
- v0.1 has not yet been built with the VEX extension or run on a robot.

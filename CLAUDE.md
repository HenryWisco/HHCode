# HHCode: context for Claude

HHCode is Henry's VEX V5 C++ template for his robotics teams. Read
`docs/SPEC.md` (what is built and what is left) and `docs/ROADMAP.md` before
proposing features.

## Decisions already made (don't reopen)

- **Platform:** VS Code + the official **VEX Robotics extension**, C++, `vex::` API.
  Not PROS.
- **Base:** JAR-Template v1.2.0 (MIT). Henry approved editing
  `include/JAR-Template/drive.h` and `src/JAR-Template/drive.cpp` for the motion
  upgrades. Every edit there is marked with a `HHCode:` comment; keep doing
  that, and don't touch the other JAR files without a reason.
- **EZ-Template** (PROS, MPL-2.0) is the feature wish list. Its code can't be
  copied in. Rebuild the behavior on the VEX API in `include/hh/` and `src/hh/`.
- **Prior art to learn from (MIT, VEX extension):** RW-Template
  (richardbwang/RW-Template) and mikLib (EthanMik/mikLib). Credit anything reused
  in `docs/CREDITS.md`.
- **Users:** students from brand new to experienced. Henry is the only maintainer.
- **Robots this season:** tank and X-drive, IMU, tracking wheels; lifts, claws,
  pistons, roller intakes. Red/blue auton mirroring is needed (not built yet).
- **Distribution:** Henry puts a zip of the template on student devices. Teams
  don't use GitHub.

## Design rules

- "Supreme usability and high-end competitiveness." Every feature has an **Easy
  layer** (one line, sensible defaults) and a **Pro layer** (every constant
  overridable, usually as public settings on the same object).
- A team describes its robot in **one file**, `src/robot-config.cpp` (plus
  `extern` lines in `include/robot-config.h`). Teams also edit `src/autons/` and
  `src/subsystems/`. They never edit `JAR-Template/` or `hh/`, and rarely `main.cpp`.
- Style follows JAR: `snake_case` functions and variables, `PascalCase` classes,
  2-space indent, `/** ... */` doc comments.
- Team-facing files get step-by-step comments a first-year student can follow.
- Keep library code simple enough for one person to read and fix. No
  exceptions, no RTTI (the VEX build turns them off). C++17 (`-std=gnu++17`, clang 8).

## Gotchas found so far

- **Pistons must be declared after `Drive chassis(...)`** in `robot-config.cpp`
  (section 3 is below section 2 for this reason). JAR's `Drive` always
  constructs 3-wire encoders for tracking wheels, even when unused, and
  `to_port()` maps out-of-range ports to three-wire port A. A `digital_out`
  made later keeps its port.
- **Devices, chassis and blocks live in the same file** so they're constructed in
  order. Globals in different `.cpp` files have no guaranteed order.
- Blocks take motors directly: `hh::Spinner intake(IntakeMotor);`. The form
  `hh::Spinner intake(motor_group(IntakeMotor));` is parsed as a function
  declaration.
- `Brain.Screen.printAt(x, y, "fmt", some_char_pointer)` is ambiguous in the SDK.
  Pass the opaque flag (`true` or `false`) as the third argument.
- `PORT1` is 0, so a negative sign can't reverse it in JAR's holonomic port list.
- Nothing may be named `clamp`: JAR has a global function with that name.
- `vex::motor_group` readings (position, velocity, temperature) come from the
  **first motor only**.
- Tasks other than the autonomous/usercontrol ones keep running between match
  periods (`competition::bStopAllTasksBetweenModes` is off by default), so a
  background task started in auton is still alive in driver control.
- Unconfirmed on hardware: whether a motor's `reversed` flag and cartridge
  belong to the `motor` object or to the port. The library assumes the object.
- `vex.h` defines the global `wait(time, msec)`. Class methods can still be named `wait`.
- VEX API reference: https://api.vex.com/v5/home/cpp/index.html

## Building and testing

- Build with the VEX extension (**Build** in the bottom bar). The makefile picks
  up `src/*.cpp` three folder levels deep. Nothing in `tests/` is in that build.
- `tests/` holds a PC simulator that runs the real library against a fake VEX
  API (see `tests/README.md`). Run it after any change to `src/hh/` or
  `src/JAR-Template/`, and add scenarios for new features.
- The simulator checks logic, not hardware. Before saying something works, say
  plainly which of these is true: it builds, it passes the simulator, it has
  been run on a robot.
- On-robot check: the **Drive Test** and **Turn Test** autons, then one
  mechanism's buttons.

## Working with Henry

- He's a teacher and coach, not a full-time programmer. Lead with the answer,
  use plain language, give click-by-click steps.
- Work on a branch and open a pull request. Don't push to `main` directly
  unless Henry asks for it.
- When a feature is finished, update `docs/ROADMAP.md`, `docs/SPEC.md` (status
  table) and the student docs in the same change.

# HHCode: context for Claude

HHCode is Henry's VEX V5 C++ template for his robotics teams. Read
`docs/SPEC.md` (what we're building and in what order) and `docs/ROADMAP.md`
before proposing features.

## Decisions already made (don't reopen)

- **Platform:** VS Code + the official **VEX Robotics extension**, C++, `vex::` API.
  Not PROS.
- **Base:** JAR-Template v1.2.0 (MIT). Its files in `include/JAR-Template/` and
  `src/JAR-Template/` stay **unmodified**.
- **EZ-Template** (PROS, MPL-2.0) is the feature wish list. Its code can't be
  copied in. Rebuild the behavior on the VEX API in `include/hh/` and `src/hh/`.
- **Prior art to learn from (MIT, VEX extension):** RW-Template
  (richardbwang/RW-Template) and mikLib (EthanMik/mikLib). Credit anything reused
  in `docs/CREDITS.md`.
- **Users:** students from brand new to experienced. Henry is the only maintainer.
- **Robots this season:** tank and X-drive, IMU, tracking wheels; lifts, claws,
  pistons, roller intakes. Red/blue auton mirroring is needed.
- **Distribution:** Henry puts a zip of the template on student devices. Teams
  don't use GitHub.

## Design rules

- "Supreme usability and high-end competitiveness." Every feature has an **Easy
  layer** (one line, sensible defaults) and, later, a **Pro layer** (every
  constant overridable).
- A team describes its robot in **one file**, `src/robot-config.cpp`. Teams edit
  `robot-config`, `main.cpp`, `autons.cpp`, `src/autons/`, `src/subsystems/`.
  They never edit `JAR-Template/` or `hh/`.
- Style follows JAR: `snake_case` functions and variables, `PascalCase` classes,
  2-space indent, `/** ... */` doc comments above each function with `@param`.
- Team-facing files get step-by-step comments a first-year student can follow.
  Mark example values with `CHANGE ME`.
- Keep library code simple enough for one person to read and fix. No
  exceptions, no RTTI (the VEX build turns them off). C++17 (`-std=gnu++17`, clang 8).

## Gotchas found so far

- **Pistons must be declared after `Drive chassis(...)`** in `robot-config.cpp`.
  JAR's `Drive` always constructs 3-wire encoders for tracking wheels, even when
  unused, and `to_port()` maps out-of-range ports to three-wire port A. A
  `digital_out` made later keeps its port.
- **Devices, chassis and blocks live in the same file** so they're constructed in
  order. Globals in different `.cpp` files have no guaranteed order.
- Declare blocks as `hh::Spinner intake = hh::Spinner(motor_group(IntakeMotor));`.
  The form `hh::Spinner intake(motor_group(IntakeMotor));` is parsed as a function
  declaration.
- `Brain.Screen.printAt(x, y, "fmt", some_char_pointer)` is ambiguous in the SDK.
  Pass `false` (the opaque flag) as the third argument.
- `PORT1` is 0, so a negative sign can't reverse it in JAR's holonomic port list.
- `vex.h` defines the global `wait(time, msec)`. Class methods can still be named `wait`.
- VEX API reference: https://api.vex.com/v5/home/cpp/index.html

## Building and testing

- Build with the VEX extension (**Build** in the bottom bar). The makefile picks
  up `src/*.cpp` three folder levels deep.
- There are no automated tests yet. Before saying something works, build it, and
  say plainly whether it has been run on a robot.
- On-robot check: the **Test: drive** and **Test: turn** autons, then
  **Example: drive + mechanisms**.

## Working with Henry

- He's a teacher and coach, not a full-time programmer. Lead with the answer,
  use plain language, give click-by-click steps.
- Work on a branch and open a pull request. Don't push to `main` directly.
- When a feature is finished, update `docs/ROADMAP.md`, `docs/SPEC.md` (status
  table) and the student docs in the same change.

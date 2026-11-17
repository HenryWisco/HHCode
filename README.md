# HHCode V5 Template

A VEX V5 C++ template for our teams, built for **VS Code + the official VEX
Robotics extension**. It starts from [JAR-Template](https://jacksonarearobotics.github.io/JAR-Template/)
(v1.2.0) and adds the best parts of [EZ-Template](https://ez-robotics.github.io/EZ-Template/),
plus our own custom features.

> **Status:** skeleton. Right now this is JAR-Template v1.2.0 with our folder
> layout, VS Code config and docs. EZ-style features get added next. See
> [docs/ROADMAP.md](docs/ROADMAP.md).

## Quick start (students)

1. Follow [docs/SETUP.md](docs/SETUP.md) once per computer.
2. Get your team's copy of this repo (your coach will tell you how) and open it in VS Code.
3. Set your motor ports in `src/robot-config.cpp` / `include/robot-config.h`.
4. Set your drivetrain in `src/main.cpp` (look for the `Drive chassis(...)` block).
5. Click **Build**, then **Download** in the VEX bar at the bottom of VS Code.

## Folder layout

```
.vscode/                 Shared VS Code + VEX extension settings (don't edit)
include/
  JAR-Template/          JAR-Template library headers (upstream, untouched)
  hh/                    Our library additions (EZ ports + custom features)
  subsystems/            One header per mechanism (intake.h, lift.h, ...)
  robot-config.h         Motor / sensor declarations
  autons.h               Auton function declarations
  vex.h                  Main include
src/
  JAR-Template/          JAR-Template library source (upstream, untouched)
  hh/                    Our library additions
  subsystems/            One .cpp per mechanism
  autons/                Auton routines (one file per routine/side)
  autons.cpp             JAR example autons (will move into autons/)
  robot-config.cpp       Motor / sensor definitions (ports, reversed, gear cartridge)
  main.cpp               Chassis setup, pre-auton, autonomous, driver control
vex/                     VEX build scripts (don't edit)
docs/                    Setup guide, roadmap, credits
makefile                 Build file used by the VEX extension
```

**Rule of thumb:** teams edit `robot-config`, `main.cpp`, `subsystems/` and
`autons/`. Library folders (`JAR-Template/`, `hh/`) are changed only in this
template repo so every team gets the same fixes.

## Docs

- [Student setup guide](docs/SETUP.md)
- [Roadmap: what we're adding from EZ-Template](docs/ROADMAP.md)
- [Credits and licenses](docs/CREDITS.md)
- JAR-Template docs: https://jacksonarearobotics.github.io/JAR-Template/

# HHCode V5 Template

A VEX V5 C++ template for our teams, built for **VS Code + the official VEX
Robotics extension**. It starts from [JAR-Template](https://jacksonarearobotics.github.io/JAR-Template/)
(v1.2.0) and adds the best parts of [EZ-Template](https://ez-robotics.github.io/EZ-Template/),
plus our own custom features.

> **Status:** in progress. The features in the roadmap are written, build, and
> pass the PC simulator tests, but have not been run on a robot yet. See
> [docs/ROADMAP.md](docs/ROADMAP.md).

## Quick start (students)

1. Follow [docs/SETUP.md](docs/SETUP.md) once per computer.
2. Open your team's copy of this folder in VS Code (**File → Open Folder**).
3. Follow [docs/GETTING-STARTED.md](docs/GETTING-STARTED.md) to set up your
   robot, buttons and first auton. Everything about your robot goes in
   `src/robot-config.cpp`.
4. Click **Build**, then **Download** in the VEX bar at the bottom of VS Code.

## Folder layout

```
.vscode/                 Shared VS Code + VEX extension settings (don't edit)
include/
  JAR-Template/          JAR-Template library headers (our additions are marked "HHCode:")
  hh/                    Our library additions (EZ ports + custom features)
  subsystems/            Headers for custom mechanisms the blocks don't cover
  robot-config.h         Motor / sensor / mechanism declarations
  autons.h               Auton function declarations
  vex.h                  Main include
src/
  JAR-Template/          JAR-Template library source (our additions are marked "HHCode:")
  hh/                    Our library additions
  subsystems/            Source for custom mechanisms
  autons/                Auton routines, PID constants, and the auton list
  robot-config.cpp       Everything about your robot: ports, chassis, mechanisms, buttons
  main.cpp               Start-up, autonomous and driver control (rarely edited)
vex/                     VEX build scripts (don't edit)
tests/                   PC simulator and tests for the library (not part of the robot program)
docs/                    Guides, roadmap, credits
makefile                 Build file used by the VEX extension
```

**Rule of thumb:** teams edit `robot-config`, `subsystems/` and `autons/`. Library folders (`JAR-Template/`, `hh/`) are changed only in this
template repo so every team gets the same fixes.

## Docs

- [Student setup guide](docs/SETUP.md): installing VS Code and the VEX extension
- [Getting started](docs/GETTING-STARTED.md): configure your robot, buttons and first auton
- [Chassis recipes](docs/CONFIG-RECIPES.md): copy-paste setups for tank, mecanum and X-drive
- [Guide](docs/GUIDE.md): every feature and setting
- [Roadmap: what we're adding from EZ-Template](docs/ROADMAP.md)
- [Spec and status table](docs/SPEC.md) and [tests](tests/README.md), for maintainers
- [Credits and licenses](docs/CREDITS.md)
- JAR-Template docs: https://jacksonarearobotics.github.io/JAR-Template/

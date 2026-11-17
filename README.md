# HHCode V5 Template

A VEX V5 C++ template for our teams, built for **VS Code + the official VEX
Robotics extension**. It starts from [JAR-Template](https://jacksonarearobotics.github.io/JAR-Template/)
(v1.2.0) and adds the best parts of [EZ-Template](https://ez-robotics.github.io/EZ-Template/),
plus our own custom features.

> **Version 0.1.** JAR-Template's motion code, plus a single commented config
> file, mechanism blocks (intake, lift, pistons) and one-line button bindings.
> More EZ-Template features are on the way. See [docs/ROADMAP.md](docs/ROADMAP.md).

## Quick start (students)

1. Follow [docs/SETUP.md](docs/SETUP.md) once per computer.
2. Open this folder in VS Code (**File → Open Folder**).
3. Follow [docs/GETTING-STARTED.md](docs/GETTING-STARTED.md) to set up your robot, buttons and first auton.

## Folder layout

```
.vscode/                 Shared VS Code + VEX extension settings (don't edit)
include/
  JAR-Template/          JAR-Template library headers (don't edit)
  hh/                    HHCode library headers: Spinner, Positioner, Pneumatic, bindings (don't edit)
  subsystems/            Headers for your own custom mechanism code
  robot-config.h         "extern" line for everything in robot-config.cpp
  autons.h               A line for each auton function
  vex.h                  Main include
src/
  JAR-Template/          JAR-Template library source (don't edit)
  hh/                    HHCode library source (don't edit)
  subsystems/            Your own custom mechanism code
  autons/                Your autons, one file each
  autons.cpp             PID constants and JAR's test autons
  robot-config.cpp       YOUR ROBOT: ports, chassis, mechanisms
  main.cpp               Auton list, buttons, lift heights, drive style
vex/                     VEX build scripts (don't edit)
docs/                    Guides
makefile                 Build file used by the VEX extension
```

**Rule of thumb:** teams edit `robot-config`, `main.cpp`, `autons.cpp`,
`subsystems/` and `autons/`. Library folders (`JAR-Template/`, `hh/`) are changed only in this
template repo so every team gets the same fixes.

## Docs

- [Student setup guide](docs/SETUP.md): installing VS Code and the VEX extension
- [Getting started](docs/GETTING-STARTED.md): configure your robot, buttons and first auton
- [Chassis recipes](docs/CONFIG-RECIPES.md): copy-paste setups for tank and X-drive
- [Roadmap: what we're adding from EZ-Template](docs/ROADMAP.md)
- [Credits and licenses](docs/CREDITS.md)
- JAR-Template docs: https://jacksonarearobotics.github.io/JAR-Template/

# Tests

These run the real HHCode and JAR-Template source on a PC against a simulated
robot, so the library can be checked without a Brain. Nothing in this folder is
part of the robot program.

## What it is

- `sim/vex/v5.h` is a stand-in for the VEX API: motors, sensors, controller,
  Brain screen, SD card, competition switch.
- `sim/sim.cpp` is the simulated robot. It has a virtual clock (a 15 second
  auton runs in milliseconds and gives the same result every time) and simple
  physics for tank, mecanum and X-drive chassis.
- `sim/robot.cpp` describes four test robots. It takes the place of
  `src/robot-config.cpp`.
- `sim/tests.cpp` holds the scenarios: boot, the auton selector, a full match,
  every chassis motion, driver control styles, each mechanism block, the tuner.

## Running

You need a C++17 compiler on the PC (the VEX extension's compiler only builds
for the Brain).

```
sh tests/run.sh          # Linux, macOS, Git Bash
.\tests\run.ps1          # Windows PowerShell
```

On Windows without a compiler, the Zig compiler installs into this folder
with Python and nothing else (delete `tests/.venv` to remove it):

```
python -m venv tests\.venv
tests\.venv\Scripts\python.exe -m pip install ziglang
.\tests\run.ps1 -Compiler "tests\.venv\Scripts\python.exe -m ziglang c++"
```

Each robot prints its tests and a pass count. A failure shows the line in
`tests.cpp` and the value it got.

## Limits

The simulator checks logic: signs, units, state machines, task handling, exit
conditions. It does not know your robot's weight, friction or battery, so PID
constants still have to be tuned on the real robot, and anything about the
real Brain's timing or screen can only be confirmed there.

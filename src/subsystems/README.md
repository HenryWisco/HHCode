# src/subsystems

One `.cpp` per mechanism on the robot, e.g. `intake.cpp` with `intake.h` in
`include/subsystems/`. Each subsystem owns its motors/pistons and exposes simple
functions (`intake_spin()`, `clamp_toggle()`) that autons and driver control call.
Files here are picked up by the makefile automatically.

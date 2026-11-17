# src/subsystems

Most mechanisms are one line in `src/robot-config.cpp` using a block
(`hh::Spinner`, `hh::Positioner`, `hh::Velocity`, `hh::Pneumatic`). This folder
is for mechanisms the blocks don't cover: write a class that extends
`hh::Subsystem`, with its header in `include/subsystems/`. See docs/GUIDE.md.
Files here are picked up by the makefile automatically.

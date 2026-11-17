# src/subsystems

Most mechanisms don't need any code here. Set them up as `hh::Spinner`,
`hh::Positioner` or `hh::Pneumatic` blocks in `src/robot-config.cpp`.

This folder is for anything the blocks can't do, like color sorting or a
mechanism with its own special logic. Put the `.cpp` here and its header in
`include/subsystems/`. Files here are built automatically.

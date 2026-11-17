# Credits and Licenses

This template builds on other people's open-source work. Keep these notices
in every copy.

## JAR-Template

- Source: https://github.com/JacksonAreaRobotics/JAR-Template (v1.2.0)
- Docs: https://jacksonarearobotics.github.io/JAR-Template/
- Files: `include/JAR-Template/`, `src/JAR-Template/`, and the starting
  versions of `src/main.cpp`, `src/autons.cpp`, `src/robot-config.cpp`,
  `include/*.h`, `makefile`, `vex/`
- License: MIT
- Modified: `drive.h` and `drive.cpp` contain HHCode additions, each marked
  with a `HHCode:` comment.

```
MIT License

Copyright (c) 2023 2775Josh

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## EZ-Template

- Source: https://github.com/EZ-Robotics/EZ-Template (latest stable v3.2.2)
- Docs: https://ez-robotics.github.io/EZ-Template/
- License: MPL-2.0. **No EZ-Template code is copied into this repo.** Features
  listed in ROADMAP.md are reimplemented on the VEX API. If we ever copy an
  EZ-Template source file, that file must keep its MPL-2.0 header and stay
  under MPL-2.0.

## Joystick curve

The driver joystick curve in `src/hh/driver.cpp` uses the formula published by
VEX team 5225A (the E-Bots PiLons), which EZ-Template also uses. Only the formula
is used; the code is our own.

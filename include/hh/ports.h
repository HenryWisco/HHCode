#pragma once

namespace hh {

/**
 * Port check. Knows what is plugged into each of the Brain's 21 smart ports.
 *
 * - Tap the title bar of the auton screen to see every port: what is plugged
 *   in and how hot each motor is. Tap again to go back.
 * - If something that was plugged in when the program started stops
 *   answering (a cable pulled out, a dead motor), the controller rumbles and
 *   shows which port.
 */
namespace ports {

/** Turn off to stop the controller warning about unplugged ports. */
extern bool warn_unplugged;

/** What is on a port right now, e.g. "Motor", "IMU", or "" for nothing. Ports are 1 to 21. */
const char* name(int port);
/** True if the port had a device when the program started and has none now. */
bool lost(int port);
/** Motor temperature in degrees C, or 0 if the port isn't a motor. */
float temperature(int port);

/** Draws the port grid on the Brain. The selector calls this for you. */
void draw();
/** Remembers what is plugged in. hh::init() calls this for you. */
void start();
/** Looks for unplugged ports. The background task calls this for you. */
void check();

}  // namespace ports
}  // namespace hh

#pragma once
#include <functional>
#include <stdint.h>

namespace hh {

/**
 * Base class for every mechanism. The built-in blocks (Spinner, Positioner,
 * Velocity, Pneumatic) all extend it. To build something custom, extend it
 * yourself and write update(): it gets called every 10 ms in the background
 * during both autonomous and driver control.
 */
class Subsystem {
public:
  Subsystem();
  virtual ~Subsystem() {}
  Subsystem(const Subsystem&) = delete;
  Subsystem& operator=(const Subsystem&) = delete;

  /** Shown on the controller in warnings, e.g. "HOT: Intake 56C". */
  const char* name = "Mechanism";

  /** Runs once from hh::init(), before the match starts. */
  virtual void init() {}
  /** Runs every 10 ms while the robot is enabled. */
  virtual void update() {}
  /** Runs once when driver control begins. */
  virtual void on_driver_start() {}
  /** Runs once each time the robot is disabled (between match periods). */
  virtual void on_disable() {}
  /** Hottest motor in degrees C, or 0 if this mechanism has no motors. */
  virtual float temperature() { return 0; }
};

/** All subsystems that exist, in the order they were created. */
int subsystem_count();
Subsystem* subsystem_at(int index);

/**
 * Runs `action` once each time `condition` goes from false to true. Checked
 * every 10 ms while the robot is enabled, in autonomous and driver control.
 *
 *   hh::when([] { return IntakeSwitch.pressing(); }, [] { intake.stop(); });
 */
void when(std::function<bool()> condition, std::function<void()> action);

/**
 * Call once in pre_auton(), after configure_robot() and register_autons().
 * Calibrates the inertial sensor, sets pistons to their start state, and starts
 * the background task and the auton selector.
 */
void init();

/** Milliseconds since the program started. */
uint32_t now_ms();

/** True when the robot is allowed to move (not disabled by field control). */
bool robot_enabled();

}  // namespace hh

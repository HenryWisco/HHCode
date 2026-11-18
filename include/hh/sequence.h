#pragma once
#include <functional>
#include "hh/subsystem.h"
#include "hh/buttons.h"

namespace hh {

/**
 * A one-button sequence (a "macro"): several steps that run one after another
 * when the driver presses a button. You write the steps the same way you
 * write an auton, top to bottom, with waits in between.
 *
 *   // Section 3 of robot-config.cpp
 *   hh::Sequence score_high([] {
 *     lift.go_to("high");
 *     lift.wait();              // wait for the lift to get there
 *     intake.reverse();
 *     wait(400, msec);
 *     intake.stop();
 *     lift.go_to("down");
 *   });
 *
 *   // in configure_robot():
 *   score_high.bind(Controller.ButtonB);   // press to start, press again to cancel
 *
 * The steps run in the background, so the driver can keep driving while they
 * happen. Up to 8 sequences.
 *
 * In an auton, score_high.start() runs it in the background while the robot
 * drives, and score_high.wait() pauses until it is done.
 */
class Sequence : public Subsystem {
public:
  /** `steps` is the list of things to do, written like an auton. */
  Sequence(std::function<void()> steps);

  // ---- Settings (change these in configure_robot()) ----

  /**
   * Runs when the sequence is cancelled part way through. Use it to put the
   * robot back in a safe state:
   *
   *   score_high.on_cancel = [] { intake.stop(); };
   */
  std::function<void()> on_cancel;

  /**
   * Set true if the steps move the chassis (chassis.drive_distance and so on).
   * The joysticks are then ignored while the sequence runs, and moving a
   * stick cancels it so the driver always gets control back.
   */
  bool uses_chassis = false;

  /** How far (percent) a stick must move to cancel a uses_chassis sequence. */
  static float cancel_stick;

  // ---- Commands ----

  /** Starts the steps from the top. Does nothing if it is already running. */
  void start();
  /** Stops the steps where they are and runs on_cancel. */
  void cancel();
  /** Waits until the steps have finished. For autons. */
  void wait();
  bool is_running() const { return running_; }

  /** Press to start. Press again while it is running to cancel. */
  void bind(Button button);
  /** Runs while the button is held. Letting go cancels it. */
  void bind_hold(Button button);

  /** Sequences never carry on through a disable (between match periods). */
  void on_disable() override;

  /**
   * True while a uses_chassis sequence is running, so the joysticks should be
   * left alone. Cancels that sequence if the driver is moving a stick.
   * hh::driver::update() calls this for you.
   */
  static bool chassis_busy();

  /** Used by the background task that runs the steps. Don't call it. */
  void run_steps();

private:
  std::function<void()> steps_;
  vex::task task_;
  int slot_ = -1;          // which of the 8 background task functions this one uses
  bool running_ = false;
};

}  // namespace hh

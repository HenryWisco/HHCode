#include "vex.h"

namespace hh {

float Sequence::cancel_stick = 20;

// Every sequence that exists. A plain array (not a vector) so sequences
// declared as globals can register themselves before main() starts.
static const int MAX_SEQUENCES = 8;
static Sequence* sequences[MAX_SEQUENCES];
static int sequence_total = 0;

// A VEX task is started from a plain function that takes no arguments, and
// the Brain tells tasks apart by which function they run. So each sequence
// needs a function of its own: entry<0>, entry<1>, ... entry<7>. Each one
// just runs the steps of the sequence in that slot.
template <int SLOT>
static int entry() {
  sequences[SLOT]->run_steps();
  return 0;
}

static int (*const entries[MAX_SEQUENCES])() = {entry<0>, entry<1>, entry<2>, entry<3>,
                                               entry<4>, entry<5>, entry<6>, entry<7>};

Sequence::Sequence(std::function<void()> steps) : steps_(steps) {
  name = "Sequence";
  if (sequence_total < MAX_SEQUENCES) {
    slot_ = sequence_total;
    sequences[sequence_total++] = this;
  }
}

// Runs in the sequence's own background task.
void Sequence::run_steps() {
  if (steps_) steps_();
  running_ = false;
}

void Sequence::start() {
  // slot_ is -1 for a ninth sequence: there is no task function left for it.
  if (running_ || slot_ < 0 || !robot_enabled()) return;
  running_ = true;
  task_ = vex::task(entries[slot_]);
}

void Sequence::cancel() {
  if (!running_) return;
  // Stopping the task ends the steps wherever they are, even in the middle
  // of a wait.
  task_.stop();
  running_ = false;
  if (uses_chassis) {
    // The steps may have been part way through a chassis motion. Stop any
    // background motion and take the power off the drive.
    chassis.cancel_motion();
    chassis.drive_stop(vex::brakeType::coast);
  }
  if (on_cancel) on_cancel();
}

void Sequence::wait() {
  while (running_) vex::task::sleep(10);
}

void Sequence::bind(Button button) {
  on_press(button, [this] {
    if (running_) cancel(); else start();
  });
}

void Sequence::bind_hold(Button button) {
  while_held(button, [this] { start(); }, [this] { cancel(); });
}

void Sequence::on_disable() { cancel(); }

bool Sequence::chassis_busy() {
  for (int i = 0; i < sequence_total; i++) {
    Sequence* sequence = sequences[i];
    if (!sequence->running_ || !sequence->uses_chassis) continue;

    // The driver always wins: moving any stick takes the robot back.
    bool stick_moved = fabs((float)Controller.Axis1.position()) > cancel_stick ||
                       fabs((float)Controller.Axis2.position()) > cancel_stick ||
                       fabs((float)Controller.Axis3.position()) > cancel_stick ||
                       fabs((float)Controller.Axis4.position()) > cancel_stick;
    if (!stick_moved) return true;
    sequence->cancel();
  }
  return false;
}

}  // namespace hh

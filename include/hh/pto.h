#pragma once
#include <vector>
#include "hh/subsystem.h"
#include "hh/buttons.h"
#include "hh/pneumatic.h"

namespace hh {

/**
 * A PTO (power take-off): drive motors that a piston can shift over to run a
 * mechanism instead, like a hang or a lift. Tank drives only.
 *
 * In robot-config.cpp, leave the PTO motors OUT of the chassis's left and
 * right motors, and give them to the Pto and to the mechanism they power:
 *
 *   motor LeftPto = motor(PORT5, ratio6_1, true);
 *   motor RightPto = motor(PORT6, ratio6_1, false);
 *   ...
 *   hh::Pneumatic pto_piston(Brain.ThreeWirePort.D);
 *   hh::Spinner hang(LeftPto, RightPto);
 *   hh::Pto pto(LeftPto, RightPto);
 *
 *   // in configure_robot():
 *   pto.use_piston(pto_piston);        // piston extended = motors on the drive
 *   pto.controls(hang);                // hang only works while the PTO is on it
 *   pto.bind_toggle(Controller.ButtonX);
 *
 *   // in an auton:
 *   pto.to_mechanism();
 *   hang.spin();
 *
 * While the PTO is on the drive, its motors do whatever their side of the
 * drive does (driver control and every chassis motion), and the mechanism
 * ignores commands. PTO motors must be reversed the same way as drive motors:
 * forward drives the robot forward.
 */
class Pto : public Subsystem {
public:
  /** One PTO motor on each side. */
  Pto(vex::motor& left, vex::motor& right);
  /** Two PTO motors on each side. */
  Pto(vex::motor& left_a, vex::motor& left_b, vex::motor& right_a, vex::motor& right_b);

  /** Where the PTO is when the program starts: true = on the drive. */
  bool start_on_drive = true;
  /** How long to let the motors go slack while the piston shifts (ms). */
  int shift_time = 0;

  /** The piston that does the shifting, and which way round it works. */
  void use_piston(Pneumatic& piston, bool extended_is_drive = true);
  /** The mechanism the PTO motors power. It is paused while they are driving. */
  void controls(Subsystem& mechanism);

  /** Motors join the drive. */
  void to_drive();
  /** Motors leave the drive and power the mechanism. */
  void to_mechanism();
  void toggle();
  bool is_driving() const { return driving_; }

  /** Each press of the button switches between drive and mechanism. */
  void bind_toggle(Button button);

  void init() override;
  float temperature() override;

private:
  void apply(bool driving);

  std::vector<vex::motor*> left_;
  std::vector<vex::motor*> right_;
  Pneumatic* piston_ = nullptr;
  bool extended_is_drive_ = true;
  Subsystem* mechanism_ = nullptr;
  bool driving_ = false;
};

}  // namespace hh

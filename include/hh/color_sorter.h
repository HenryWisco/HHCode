#pragma once
#include "hh/subsystem.h"
#include "hh/buttons.h"
#include "hh/pneumatic.h"
#include "hh/spinner.h"

namespace hh {

enum Color { NO_COLOR, RED, BLUE };

/**
 * Throws out game pieces of the wrong color, using an optical sensor that
 * looks at the pieces as they pass.
 *
 *   optical RingSensor = optical(PORT7);              // section 1
 *   hh::ColorSorter sorter(RingSensor);               // section 3
 *
 *   // in configure_robot(): how to throw a piece out. Pick one.
 *   sorter.eject_with(ejector);                       // fire a piston
 *   sorter.eject_with(intake, -100);                  // or run the intake backwards
 *
 *   // in an auton, or once you know your alliance:
 *   sorter.keep(hh::RED);                             // blue pieces get thrown out
 *
 * Use  hh::Color c = sorter.seen();  and the Brain screen to check what the
 * sensor reads, and adjust the hue ranges below if it gets colors wrong.
 */
class ColorSorter : public Subsystem {
public:
  ColorSorter(vex::optical& sensor);

  // ---- Settings (change these in configure_robot()) ----

  /** Hue ranges in degrees (0 to 360). Red wraps around 0, so min is bigger than max. */
  float red_hue_min = 340;
  float red_hue_max = 25;
  float blue_hue_min = 180;
  float blue_hue_max = 260;
  /** Only trust the color when the sensor says something is close to it. */
  bool use_proximity = true;
  /** Brightness of the sensor's light, 0 to 100. 0 leaves it off. */
  int light_power = 100;
  /** The wrong color must be seen for this long (ms) before it counts. */
  int confirm_time = 20;
  /** Time (ms) from seeing a piece to throwing it out: how long it takes to reach the ejector. */
  int eject_delay = 0;
  /** How long (ms) the piston stays out or the motors run the other way. */
  int eject_time = 150;

  // ---- Commands ----

  /** Sets which color to keep; the other is thrown out. hh::NO_COLOR turns sorting off. */
  void keep(Color color);
  Color keeping() const { return keep_; }
  /** Pauses or resumes sorting without forgetting the color. */
  void set_active(bool active);
  bool is_active() const { return active_ && keep_ != NO_COLOR; }
  /** What the sensor sees right now. */
  Color seen();
  /** How many pieces have been thrown out since the program started. */
  int rejected() const { return rejected_; }
  bool is_ejecting() const { return ejecting_; }

  /** Throws a piece out by extending a piston for eject_time. */
  void eject_with(Pneumatic& piston);
  /** Throws a piece out by running a spinner at `percent` for eject_time. 0 stops it instead. */
  void eject_with(Spinner& spinner, float percent = -100);
  /** Anything else: `start` runs to throw the piece out, `finish` runs eject_time later. */
  void eject_with(Action start, Action finish);

  /** Each press pauses or resumes sorting. */
  void bind_toggle(Button button);

  void init() override;
  void update() override;
  void on_disable() override;

private:
  static bool in_range(float hue, float min, float max);
  void end_eject();

  // The sensor, the color to keep, and how to throw a piece out.
  vex::optical* sensor_;
  Color keep_ = NO_COLOR;
  bool active_ = true;
  Action start_;
  Action finish_;

  // Where the current piece is in the steps (see color_sorter.cpp):
  // seen wrong -> flagged -> pending (waiting eject_delay) -> ejecting.
  // The times are in ms since the program started.
  bool wrong_ = false;
  bool flagged_ = false;
  uint32_t wrong_since_ = 0;
  bool pending_ = false;
  uint32_t pending_at_ = 0;
  bool ejecting_ = false;
  uint32_t eject_end_ = 0;
  int rejected_ = 0;
};

}  // namespace hh

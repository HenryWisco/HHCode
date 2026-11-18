#include "vex.h"

namespace hh {
namespace selector {

bool show_on_brain = true;
bool use_controller = true;

struct Auton {
  const char* name;
  const char* description;
  void (*routine)();
  bool mirrored;
};

static const int MAX_AUTONS = 24;
static Auton autons[MAX_AUTONS];
static int total = 0;
static int current = 0;

static const char* SAVE_FILE = "hh_auton.txt";

void add(const char* name, const char* description, void (*routine)()) {
  if (total < MAX_AUTONS) {
    autons[total++] = {name, description, routine, false};
  }
}

void add_mirrored(const char* name, const char* mirrored_name, const char* description, void (*routine)()) {
  add(name, description, routine);
  if (total < MAX_AUTONS) {
    autons[total++] = {mirrored_name, description, routine, true};
  }
}

bool mirrored() { return chassis.is_mirrored(); }

int selected() { return current; }

int count() { return total; }

const char* selected_name() { return total > 0 ? autons[current].name : "No autons"; }

// The name is saved rather than the number, so reordering the list doesn't
// silently change which auton runs.
static void save() {
  if (total == 0 || !Brain.SDcard.isInserted()) return;
  const char* name = autons[current].name;
  Brain.SDcard.savefile(SAVE_FILE, (uint8_t*)name, strlen(name));
}

static void load() {
  if (!Brain.SDcard.isInserted() || !Brain.SDcard.exists(SAVE_FILE)) return;
  char name[64] = {0};
  Brain.SDcard.loadfile(SAVE_FILE, (uint8_t*)name, sizeof(name) - 1);
  for (int i = 0; i < total; i++) {
    if (strcmp(autons[i].name, name) == 0) current = i;
  }
}

void next() {
  if (total == 0) return;
  current = (current + 1) % total;
  save();
}

void previous() {
  if (total == 0) return;
  current = (current + total - 1) % total;
  save();
}

void run_selected() {
  if (total == 0) return;
  chassis.set_mirrored(autons[current].mirrored);
  autons[current].routine();
  // Motions leave the last bit of PID power on the motors so the next one can
  // take over smoothly. After the final one, that would make the robot creep
  // for the rest of autonomous, so stop the drive here.
  chassis.wait();
  chassis.drive_stop(vex::brakeType::brake);
  chassis.set_mirrored(false);
}

// Prints `text` on up to `max_lines` lines, breaking at spaces.
static void print_wrapped(const char* text, int x, int y, int line_height, int width, int max_lines) {
  int length = strlen(text);
  int start = 0;
  for (int line = 0; line < max_lines && start < length; line++) {
    int end = start + width;
    if (end >= length) {
      end = length;
    } else {
      int space = end;
      while (space > start && text[space] != ' ') space--;
      if (space > start) end = space;
    }
    char buffer[64];
    int n = end - start;
    if (n > (int)sizeof(buffer) - 1) n = sizeof(buffer) - 1;
    memcpy(buffer, text + start, n);
    buffer[n] = 0;
    Brain.Screen.printAt(x, y + line * line_height, true, "%s", buffer);
    start = end;
    while (start < length && text[start] == ' ') start++;
  }
}

static void draw_status() {
  Brain.Screen.setFont(vex::fontType::mono20);
  Brain.Screen.setFillColor(vex::color(0x1565C0));
  Brain.Screen.setPenColor(vex::color::white);
  Brain.Screen.printAt(275, 22, "Batt %3d%% Head %5.1f", (int)Brain.Battery.capacity(),
                       chassis.get_absolute_heading());
  Brain.Screen.setFillColor(vex::color::black);
}

static void draw_screen() {
  Brain.Screen.clearScreen();

  // Title bar
  Brain.Screen.setPenColor(vex::color(0x1565C0));
  Brain.Screen.setFillColor(vex::color(0x1565C0));
  Brain.Screen.drawRectangle(0, 0, 480, 32);
  Brain.Screen.setPenColor(vex::color::white);
  Brain.Screen.setFont(vex::fontType::mono20);
  // Tapping "Ports>" or "Map>" opens that page. A missing inertial sensor is
  // the most common reason autons go wrong, so it is called out here.
  Brain.Screen.printAt(10, 22, true, "Ports>  Map>  %s%s", Brain.SDcard.isInserted() ? "" : "no SD ",
                       chassis.Gyro.installed() ? "" : "NO IMU!");
  draw_status();

  // Chosen auton
  Brain.Screen.setFillColor(vex::color::black);
  Brain.Screen.setPenColor(vex::color(0x9E9E9E));
  if (total == 0) {
    Brain.Screen.printAt(10, 60, "No autons yet");
    Brain.Screen.setPenColor(vex::color::white);
    print_wrapped("Add them in register_autons() in src/autons/auton_list.cpp", 10, 100, 24, 46, 3);
  } else {
    Brain.Screen.printAt(10, 60, "Auton %d of %d", current + 1, total);
    Brain.Screen.setPenColor(vex::color::white);
    Brain.Screen.setFont(vex::fontType::mono30);
    Brain.Screen.printAt(10, 98, true, "%s", autons[current].name);
    Brain.Screen.setFont(vex::fontType::mono20);
    Brain.Screen.setPenColor(vex::color(0xBDBDBD));
    print_wrapped(autons[current].description, 10, 128, 22, 46, 2);
  }

  // Buttons
  Brain.Screen.setPenColor(vex::color(0x424242));
  Brain.Screen.setFillColor(vex::color(0x424242));
  Brain.Screen.drawRectangle(8, 178, 228, 54);
  Brain.Screen.drawRectangle(244, 178, 228, 54);
  Brain.Screen.setPenColor(vex::color::white);
  Brain.Screen.setFont(vex::fontType::mono30);
  Brain.Screen.printAt(70, 215, "< PREV");
  Brain.Screen.printAt(306, 215, "NEXT >");
  Brain.Screen.setFont(vex::fontType::mono20);
  Brain.Screen.setFillColor(vex::color::black);
}

static void draw_controller() {
  Controller.Screen.clearLine(1);
  Controller.Screen.setCursor(1, 1);
  if (total == 0) {
    Controller.Screen.print("No autons");
  } else {
    Controller.Screen.print("%d/%d %s", current + 1, total, autons[current].name);
  }
}

static int selector_task() {
  int shown = -1;
  bool was_running_auton = false;
  bool was_touching = false;
  // Which page is on the Brain: the auton list, the ports page or the map.
  enum Page { AUTONS, PORTS, MAP };
  Page page = AUTONS;
  bool was_left = false;
  bool was_right = false;
  uint32_t last_status = 0;

  while (true) {
    bool running_auton = Competition.isEnabled() && Competition.isAutonomous();
    bool disabled_by_field = !Competition.isEnabled() &&
                             (Competition.isCompetitionSwitch() || Competition.isFieldControl());

    if (running_auton) {
      if (show_on_brain && page == MAP && !graph::owns_screen()) {
        // The map was open when the auton started: keep it up to watch the routine.
        if (now_ms() - last_status > 200) {
          last_status = now_ms();
          odom_map::draw();
        }
      } else if (!was_running_auton && show_on_brain && !graph::owns_screen()) {
        // Leave the screen to the auton so it can print its own information.
        Brain.Screen.clearScreen();
      }
      was_running_auton = true;
      shown = -1;
      vex::task::sleep(50);
      continue;
    }
    was_running_auton = false;

    // The PID graph borrows the screen until it is tapped.
    if (graph::owns_screen()) {
      shown = -1;
      was_touching = false;
      vex::task::sleep(50);
      continue;
    }

    if (show_on_brain) {
      bool touching = Brain.Screen.pressing();
      bool tapped = touching && !was_touching;
      was_touching = touching;

      // The ports page and the map: opened from the title bar, closed by any tap.
      if (page != AUTONS) {
        if (tapped) {
          page = AUTONS;
          shown = -1;
        } else if (now_ms() - last_status > (page == PORTS ? 500u : 200u)) {
          last_status = now_ms();
          if (page == PORTS) ports::draw(); else odom_map::draw();
        }
        if (page != AUTONS) {
          vex::task::sleep(20);
          continue;
        }
      } else if (tapped) {
        if (Brain.Screen.yPosition() < 36) {
          // "Map>" sits between x = 90 and 130. The rest of the bar is ports.
          int x = Brain.Screen.xPosition();
          page = (x >= 80 && x < 150) ? MAP : PORTS;
          last_status = 0;
          vex::task::sleep(20);
          continue;
        }
        if (Brain.Screen.xPosition() < 240) previous(); else next();
      }
    }

    if (use_controller && disabled_by_field) {
      bool left = Controller.ButtonLeft.pressing();
      bool right = Controller.ButtonRight.pressing();
      if (left && !was_left) previous();
      if (right && !was_right) next();
      was_left = left;
      was_right = right;
    }

    if (shown != current) {
      shown = current;
      if (show_on_brain) draw_screen();
      if (use_controller) draw_controller();
    } else if (show_on_brain && now_ms() - last_status > 250) {
      last_status = now_ms();
      draw_status();
    }

    vex::task::sleep(20);
  }
  return 0;
}

void start() {
  load();
  static vex::task task(selector_task);
}

}  // namespace selector
}  // namespace hh

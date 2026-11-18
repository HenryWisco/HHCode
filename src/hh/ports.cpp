// Port check: what is plugged into each smart port, and whether anything
// has come unplugged since the program started.
//
// The VEX C function vexDeviceGetStatus() fills an array with the kind of
// device on every port. start() keeps a copy from start-up (at_start);
// everything else compares the latest reading (now) with it.

#include "vex.h"

namespace hh {
namespace ports {

bool warn_unplugged = true;

static const int PORT_COUNT = 21;
static V5_DeviceType at_start[V5_MAX_DEVICE_PORTS];
static V5_DeviceType now[V5_MAX_DEVICE_PORTS];
static bool started = false;
static bool warned[PORT_COUNT];

// Refreshes `now` from the Brain.
static void read() { vexDeviceGetStatus(now); }

// A short name for each kind of device, to fit in a box on the screen.
static const char* type_name(V5_DeviceType type) {
  switch (type) {
    case kDeviceTypeNoSensor: return "";
    case kDeviceTypeMotorSensor: return "Motor";
    case kDeviceTypeAbsEncSensor: return "Rotate";
    case kDeviceTypeImuSensor: return "IMU";
    case kDeviceTypeDistanceSensor: return "Dist";
    case kDeviceTypeRadioSensor: return "Radio";
    case kDeviceTypeVisionSensor: return "Vision";
    case kDeviceTypeAdiSensor: return "3-Wire";
    case kDeviceTypeOpticalSensor: return "Optic";
    case kDeviceTypeGpsSensor: return "GPS";
    case kDeviceTypeAiVisionSensor: return "AI Vis";
    default: return "Other";
  }
}

// Port numbers here are 1 to 21, as printed on the Brain.
static bool valid(int port) { return port >= 1 && port <= PORT_COUNT; }

const char* name(int port) {
  if (!valid(port)) return "";
  read();
  return type_name(now[port - 1]);
}

bool lost(int port) {
  if (!valid(port) || !started) return false;
  read();
  return at_start[port - 1] != kDeviceTypeNoSensor && now[port - 1] == kDeviceTypeNoSensor;
}

float temperature(int port) {
  if (!valid(port)) return 0;
  read();
  if (now[port - 1] != kDeviceTypeMotorSensor) return 0;
  return vexDeviceMotorTemperatureGet(vexDeviceGetByIndex(port - 1));
}

void start() {
  vexDeviceGetStatus(at_start);
  for (int i = 0; i < PORT_COUNT; i++) warned[i] = false;
  started = true;
}

// Twice a second: has a port that had a device at start-up gone empty?
// Warn once per unplugging, and again only if it is plugged back in
// and then drops out again.
void check() {
  static uint32_t last_check = 0;
  uint32_t time = now_ms();
  if (!started || !warn_unplugged || time - last_check < 500) return;
  last_check = time;
  read();
  for (int i = 0; i < PORT_COUNT; i++) {
    bool gone = at_start[i] != kDeviceTypeNoSensor && now[i] == kDeviceTypeNoSensor;
    if (gone && !warned[i]) {
      warned[i] = true;
      Controller.rumble("---");
      Controller.Screen.setCursor(3, 1);
      Controller.Screen.print("PORT %d UNPLUGGED   ", i + 1);
    }
    if (!gone) warned[i] = false;   // plugged back in: warn again if it drops again
  }
}

// Draws all 21 ports as a grid of boxes: grey = empty, green = fine,
// orange = hot motor, red = was plugged in and has gone.
void draw() {
  read();
  Brain.Screen.clearScreen();
  Brain.Screen.setFont(vex::fontType::mono20);
  Brain.Screen.setFillColor(vex::color::black);
  Brain.Screen.setPenColor(vex::color::white);
  Brain.Screen.printAt(10, 22, true, "Ports  (tap to go back)");

  // 7 columns by 3 rows.
  const int cell_width = 68, cell_height = 66, left = 2, top = 34;
  for (int i = 0; i < PORT_COUNT; i++) {
    int x = left + (i % 7) * cell_width;
    int y = top + (i / 7) * cell_height;
    V5_DeviceType type = now[i];
    bool gone = started && at_start[i] != kDeviceTypeNoSensor && type == kDeviceTypeNoSensor;
    float degrees = type == kDeviceTypeMotorSensor ? vexDeviceMotorTemperatureGet(vexDeviceGetByIndex(i)) : 0;
    bool hot = driver::overheat_temp > 0 && degrees >= driver::overheat_temp;

    vex::color fill = vex::color(0x212121);                       // nothing here
    if (type != kDeviceTypeNoSensor) fill = vex::color(0x2E7D32);  // fine
    if (hot) fill = vex::color(0xEF6C00);
    if (gone) fill = vex::color(0xC62828);
    Brain.Screen.setPenColor(vex::color(0x616161));
    Brain.Screen.setFillColor(fill);
    Brain.Screen.drawRectangle(x, y, cell_width - 2, cell_height - 2);

    Brain.Screen.setPenColor(vex::color::white);
    Brain.Screen.printAt(x + 4, y + 18, false, "%d", i + 1);
    if (gone) {
      Brain.Screen.printAt(x + 4, y + 38, false, "LOST");
      Brain.Screen.printAt(x + 4, y + 58, false, "%.5s", type_name(at_start[i]));
    } else if (type != kDeviceTypeNoSensor) {
      Brain.Screen.printAt(x + 4, y + 38, false, "%.6s", type_name(type));
      if (type == kDeviceTypeMotorSensor) Brain.Screen.printAt(x + 4, y + 58, false, "%dC", (int)degrees);
    }
  }
  Brain.Screen.setFillColor(vex::color::black);
}

}  // namespace ports
}  // namespace hh

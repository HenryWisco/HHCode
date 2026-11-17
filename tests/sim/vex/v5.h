// A stand-in for the VEX V5 API, so the robot code can be compiled and run on
// a PC against a simulated robot (tests/sim/sim.cpp). Only the parts of the
// API that HHCode and JAR-Template use are here. This file is never part of
// the real robot build.
#pragma once
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

// The small part of the C API that HHCode uses to see what is plugged in.
#define V5_MAX_DEVICE_PORTS 32
typedef enum {
  kDeviceTypeNoSensor = 0,
  kDeviceTypeMotorSensor = 2,
  kDeviceTypeAbsEncSensor = 4,
  kDeviceTypeImuSensor = 6,
  kDeviceTypeDistanceSensor = 7,
  kDeviceTypeRadioSensor = 8,
  kDeviceTypeVisionSensor = 11,
  kDeviceTypeAdiSensor = 12,
  kDeviceTypeOpticalSensor = 16,
  kDeviceTypeGpsSensor = 20,
  kDeviceTypeAiVisionSensor = 29
} V5_DeviceType;
typedef struct _V5_Device* V5_DeviceT;
int32_t vexDeviceGetStatus(V5_DeviceType* buffer);
V5_DeviceT vexDeviceGetByIndex(uint32_t index);
double vexDeviceMotorTemperatureGet(V5_DeviceT device);

namespace vex {

enum class percentUnits { pct = 0 };
enum class timeUnits { sec, msec };
enum class currentUnits { amp };
enum class voltageUnits { volt, mV };
enum class rotationUnits { deg, rev, raw = 99 };
enum class velocityUnits { pct = 0, rpm, dps };
enum class temperatureUnits { celsius, fahrenheit };
enum class directionType { fwd = 0, rev };
enum class brakeType { coast, brake, hold };
enum class gearSetting { ratio36_1, ratio18_1, ratio6_1 };
enum class fontType { mono20 = 0, mono30, mono40, mono60, prop20, prop30, prop40, prop60, mono15, mono12 };
enum class controllerType { primary, partner };
enum class ledState { off, on };

constexpr percentUnits percent = percentUnits::pct;
constexpr percentUnits pct = percentUnits::pct;
constexpr timeUnits seconds = timeUnits::sec;
constexpr timeUnits sec = timeUnits::sec;
constexpr timeUnits msec = timeUnits::msec;
constexpr voltageUnits volt = voltageUnits::volt;
constexpr currentUnits amp = currentUnits::amp;
constexpr rotationUnits degrees = rotationUnits::deg;
constexpr rotationUnits deg = rotationUnits::deg;
constexpr rotationUnits rev = rotationUnits::rev;
constexpr velocityUnits rpm = velocityUnits::rpm;
constexpr velocityUnits dps = velocityUnits::dps;
constexpr temperatureUnits celsius = temperatureUnits::celsius;
constexpr directionType forward = directionType::fwd;
constexpr directionType fwd = directionType::fwd;
constexpr directionType reverse = directionType::rev;
constexpr brakeType coast = brakeType::coast;
constexpr brakeType brake = brakeType::brake;
constexpr brakeType hold = brakeType::hold;
constexpr gearSetting ratio36_1 = gearSetting::ratio36_1;
constexpr gearSetting ratio18_1 = gearSetting::ratio18_1;
constexpr gearSetting ratio6_1 = gearSetting::ratio6_1;
constexpr fontType mono20 = fontType::mono20;
constexpr fontType mono30 = fontType::mono30;
constexpr controllerType primary = controllerType::primary;
constexpr controllerType partner = controllerType::partner;

constexpr int32_t PORT1 = 0, PORT2 = 1, PORT3 = 2, PORT4 = 3, PORT5 = 4, PORT6 = 5, PORT7 = 6,
                  PORT8 = 7, PORT9 = 8, PORT10 = 9, PORT11 = 10, PORT12 = 11, PORT13 = 12,
                  PORT14 = 13, PORT15 = 14, PORT16 = 15, PORT17 = 16, PORT18 = 17, PORT19 = 18,
                  PORT20 = 19, PORT21 = 20, PORT22 = 21;

class color {
public:
  uint32_t rgb;
  color() : rgb(0) {}
  color(uint32_t value) : rgb(value) {}
  static const color black;
  static const color white;
};

class timer {
public:
  static uint32_t system();
};

class task {
  int (*_callback)(void) = nullptr;

public:
  task() {}
  task(int (*callback)(void));
  void stop();
  static void sleep(uint32_t time);
  static void yield();
};

void wait(double time, timeUnits units);

class motor {
  int32_t _index;
  gearSetting _gears;
  bool _reversed;
  double sign(directionType dir) const;

public:
  motor(int32_t index) : _index(index), _gears(gearSetting::ratio18_1), _reversed(false) {}
  motor(int32_t index, bool reverse) : _index(index), _gears(gearSetting::ratio18_1), _reversed(reverse) {}
  motor(int32_t index, gearSetting gears) : _index(index), _gears(gears), _reversed(false) {}
  motor(int32_t index, gearSetting gears, bool reverse) : _index(index), _gears(gears), _reversed(reverse) {}

  bool installed();
  void spin(directionType dir, double value, voltageUnits units);
  void spin(directionType dir, double value, velocityUnits units);
  void spin(directionType dir, double value, percentUnits) { spin(dir, value, velocityUnits::pct); }
  void stop();
  void stop(brakeType mode);
  void setStopping(brakeType mode);
  void setMaxTorque(double value, currentUnits units);
  void setPosition(double value, rotationUnits units);
  void resetPosition() { setPosition(0, rotationUnits::deg); }
  double position(rotationUnits units);
  double velocity(velocityUnits units);
  double velocity(percentUnits) { return velocity(velocityUnits::pct); }
  double voltage(voltageUnits units = voltageUnits::volt);
  double temperature(temperatureUnits units);
};

class motor_group {
  std::vector<motor> _motors;
  void _add() {}
  template <typename... Rest>
  void _add(motor& m, Rest&... rest) {
    _motors.push_back(m);
    _add(rest...);
  }

public:
  motor_group() {}
  template <typename... Rest>
  motor_group(motor& m, Rest&... rest) {
    _add(m, rest...);
  }

  int32_t count() { return (int32_t)_motors.size(); }
  void spin(directionType dir, double value, voltageUnits units) { for (motor& m : _motors) m.spin(dir, value, units); }
  void spin(directionType dir, double value, velocityUnits units) { for (motor& m : _motors) m.spin(dir, value, units); }
  void spin(directionType dir, double value, percentUnits units) { for (motor& m : _motors) m.spin(dir, value, units); }
  void stop() { for (motor& m : _motors) m.stop(); }
  void stop(brakeType mode) { for (motor& m : _motors) m.stop(mode); }
  void setStopping(brakeType mode) { for (motor& m : _motors) m.setStopping(mode); }
  void setMaxTorque(double value, currentUnits units) { for (motor& m : _motors) m.setMaxTorque(value, units); }
  void setPosition(double value, rotationUnits units) { for (motor& m : _motors) m.setPosition(value, units); }
  void resetPosition() { for (motor& m : _motors) m.resetPosition(); }
  // Like the real API, readings come from the first motor in the group.
  double position(rotationUnits units) { return _motors.empty() ? 0 : _motors[0].position(units); }
  double velocity(velocityUnits units) { return _motors.empty() ? 0 : _motors[0].velocity(units); }
  double velocity(percentUnits units) { return _motors.empty() ? 0 : _motors[0].velocity(units); }
  double voltage(voltageUnits units = voltageUnits::volt) { return _motors.empty() ? 0 : _motors[0].voltage(units); }
  double temperature(temperatureUnits units) { return _motors.empty() ? 0 : _motors[0].temperature(units); }
};

class inertial {
  int32_t _index;

public:
  inertial(int32_t index) : _index(index) {}
  bool installed();
  void calibrate();
  bool isCalibrating();
  double rotation(rotationUnits units = rotationUnits::deg);
  double heading(rotationUnits units = rotationUnits::deg);
  void setRotation(double value, rotationUnits units);
  void setHeading(double value, rotationUnits units) { setRotation(value, units); }
};

class rotation {
  int32_t _index;
  bool _reversed;

public:
  rotation(int32_t index, bool reverse = false) : _index(index), _reversed(reverse) {}
  double position(rotationUnits units);
  void setPosition(double value, rotationUnits units);
  void resetPosition() { setPosition(0, rotationUnits::deg); }
};

class optical {
  int32_t _index;

public:
  optical(int32_t index, bool enableGesture = false) : _index(index) { (void)enableGesture; }
  bool installed() { return true; }
  double hue();
  bool isNearObject();
  void setLight(ledState state);
  void setLightPower(int32_t intensity, percentUnits units = percentUnits::pct);
};

class triport {
  int32_t _index;

public:
  struct port {
    int32_t id = 0;
  };
  port Port[8];
  port& A;
  port& B;
  port& C;
  port& D;
  port& E;
  port& F;
  port& G;
  port& H;

  triport(int32_t index)
      : _index(index), A(Port[0]), B(Port[1]), C(Port[2]), D(Port[3]), E(Port[4]), F(Port[5]), G(Port[6]), H(Port[7]) {
    for (int i = 0; i < 8; i++) Port[i].id = i;
  }
  triport(const triport& other)
      : _index(other._index), A(Port[0]), B(Port[1]), C(Port[2]), D(Port[3]), E(Port[4]), F(Port[5]), G(Port[6]), H(Port[7]) {
    for (int i = 0; i < 8; i++) Port[i].id = i;
  }
};

class encoder {
  int32_t _id;

public:
  encoder(triport::port& port) : _id(port.id) {}
  double position(rotationUnits) { return 0; }
  void resetRotation() {}
};

class digital_out {
  int32_t _id;

public:
  digital_out(triport::port& port) : _id(port.id) {}
  void set(bool value);
  int32_t value();
};

class brain {
public:
  class lcd {
  public:
    void clearScreen();
    void setFont(fontType font);
    void setFillColor(const color& c);
    void setPenColor(const color& c);
    void drawRectangle(int x, int y, int width, int height);
    void drawLine(int x1, int y1, int x2, int y2);
    void setPenWidth(uint32_t width);
    void printAt(int32_t x, int32_t y, const char* format, ...);
    void printAt(int32_t x, int32_t y, bool opaque, const char* format, ...);
    bool pressing();
    int32_t xPosition();
    int32_t yPosition();
  };
  class battery {
  public:
    uint32_t capacity(percentUnits units = percentUnits::pct);
  };
  class sdcard {
  public:
    bool isInserted();
    bool exists(const char* name);
    int32_t loadfile(const char* name, uint8_t* buffer, int32_t len);
    int32_t savefile(const char* name, uint8_t* buffer, int32_t len);
  };

  lcd Screen;
  battery Battery;
  sdcard SDcard;
  triport ThreeWirePort = triport(PORT22);
};

class controller {
public:
  class button {
    int32_t _id;

  public:
    button(int32_t id) : _id(id) {}
    bool pressing() const;
  };
  class axis {
    int32_t _id;

  public:
    axis(int32_t id) : _id(id) {}
    int32_t position(percentUnits units = percentUnits::pct) const;
    int32_t value() const;
  };
  class lcd {
  public:
    void setCursor(int32_t row, int32_t col);
    void print(const char* format, ...);
    void clearScreen();
    void clearLine(int number);
    void clearLine();
  };

  controller() {}
  controller(controllerType) {}

  const button ButtonL1 = button(0);
  const button ButtonL2 = button(1);
  const button ButtonR1 = button(2);
  const button ButtonR2 = button(3);
  const button ButtonUp = button(4);
  const button ButtonDown = button(5);
  const button ButtonLeft = button(6);
  const button ButtonRight = button(7);
  const button ButtonX = button(8);
  const button ButtonB = button(9);
  const button ButtonY = button(10);
  const button ButtonA = button(11);
  const axis Axis1 = axis(1);
  const axis Axis2 = axis(2);
  const axis Axis3 = axis(3);
  const axis Axis4 = axis(4);
  lcd Screen;

  void rumble(const char* pattern);
};

class competition {
public:
  competition() {}
  void autonomous(void (*callback)(void));
  void drivercontrol(void (*callback)(void));
  bool isEnabled();
  bool isDriverControl();
  bool isAutonomous();
  bool isCompetitionSwitch();
  bool isFieldControl();
};

}  // namespace vex

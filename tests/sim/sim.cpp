// The simulated robot: a task scheduler with a virtual clock, a simple
// physics model, and the bodies of the fake VEX API in vex/v5.h.
//
// Every vex::task is a real thread, but only one runs at a time and they only
// switch when one sleeps. Time is virtual: it jumps forward to the moment the
// next task wakes, stepping the physics on the way. So a 15 second autonomous
// runs in a few milliseconds and gives the same answer every time.
#include "sim.h"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <thread>

namespace sim {

/*---------------------------------------------------------------------------*/
/*  Scheduler                                                                */
/*---------------------------------------------------------------------------*/

struct Thread {
  const void* key = nullptr;
  const char* label = "";
  std::function<void()> body;
  double wake = 0;
  uint64_t order = 0;
  bool dead = false;
  bool go = false;
  std::condition_variable cv;
};

static std::mutex mutex;
static std::vector<Thread*> threads;
static Thread* current = nullptr;
static double now_ms = 0;
static uint64_t order_counter = 0;
static uint64_t switches_without_time = 0;
static std::atomic<uint64_t> progress{0};
static thread_local std::unique_lock<std::mutex>* my_lock = nullptr;

static void physics_step(double dt);

static void advance_to(double time) {
  while (now_ms + 1e-9 < time) {
    double step = time - now_ms < 1.0 ? time - now_ms : 1.0;
    physics_step(step / 1000.0);
    now_ms += step;
  }
  now_ms = time;
}

void finish(int code) {
  fflush(stdout);
  fflush(stderr);
  std::_Exit(code);
}

// Hands control to whichever task should run next. Called by the running
// task, which then waits here until it is its turn again.
static void switch_from(Thread* self) {
  progress++;
  Thread* best = nullptr;
  for (Thread* t : threads) {
    if (t->dead) continue;
    if (best == nullptr || t->wake < best->wake || (t->wake == best->wake && t->order < best->order)) best = t;
  }
  if (best == nullptr) {
    printf("SIM: every task has ended\n");
    finish(3);
  }
  if (best->wake > now_ms) {
    advance_to(best->wake);
    switches_without_time = 0;
    if (now_ms > 300000) {
      printf("SIM: the test ran for more than 5 minutes of robot time\n");
      finish(3);
    }
  } else if (++switches_without_time > 2000000) {
    printf("SIM HANG at %.0f ms: tasks are running without ever sleeping (last: %s)\n", now_ms, self->label);
    finish(3);
  }
  current = best;
  if (best == self) return;
  self->go = false;
  best->go = true;
  best->cv.notify_one();
  if (self->dead) return;
  self->cv.wait(*my_lock, [self] { return self->go; });
}

static void sleep_ms(double ms) {
  Thread* self = current;
  self->wake = now_ms + (ms > 0 ? ms : 0);
  self->order = ++order_counter;
  switch_from(self);
}

static Thread* spawn(const void* key, const char* label, std::function<void()> body) {
  Thread* t = new Thread;
  t->key = key;
  t->label = label;
  t->body = body;
  t->wake = now_ms;
  t->order = ++order_counter;
  threads.push_back(t);
  std::thread([t] {
    std::unique_lock<std::mutex> lock(mutex);
    my_lock = &lock;
    t->cv.wait(lock, [t] { return t->go; });
    t->body();
    t->dead = true;
    switch_from(t);
  }).detach();
  return t;
}

static void kill(Thread* t) {
  if (t->dead) return;
  t->dead = true;
  if (t == current) {
    // A task stopping itself never runs again.
    switch_from(t);
    t->cv.wait(*my_lock, [] { return false; });
  }
}

static void kill_key(const void* key) {
  Thread* self = nullptr;
  for (Thread* t : threads) {
    if (t->key != key || t->dead) continue;
    if (t == current) self = t; else kill(t);
  }
  if (self != nullptr) kill(self);
}

static void watchdog() {
  uint64_t last = 0;
  int stuck_seconds = 0;
  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    uint64_t value = progress.load();
    stuck_seconds = value == last ? stuck_seconds + 1 : 0;
    last = value;
    if (stuck_seconds >= 20) {
      printf("SIM HANG at %.0f ms: a task is stuck in a loop that never sleeps\n", now_ms);
      finish(3);
    }
  }
}

double now() { return now_ms; }

void run(double ms) { sleep_ms(ms); }

/*---------------------------------------------------------------------------*/
/*  Robot state                                                              */
/*---------------------------------------------------------------------------*/

Robot robot;
bool sd_inserted = true;
bool imu_installed = true;
V5_DeviceType devices[V5_MAX_DEVICE_PORTS] = {};
double optical_hue = 100;
bool optical_near = false;
bool optical_light = false;
static int brain_line_total = 0;
static bool brain_line_off_screen = false;
int brain_lines() { return brain_line_off_screen ? -1 : brain_line_total; }

static MotorState motors[22];
static Mode current_mode = DRIVER;
static bool field_connected = false;
static void (*auton_callback)() = nullptr;
static void (*driver_callback)() = nullptr;
static int competition_key = 0;

static double imu_offset_deg = 0;
static double imu_calibrating_until = -1;
static double tracker_forward_in = 0;
static double tracker_sideways_in = 0;
static double tracker_forward_zero = 0;
static double tracker_sideways_zero = 0;

static double axes[5] = {0, 0, 0, 0, 0};
static bool buttons[12] = {false};
static const char* button_names[12] = {"L1", "L2", "R1", "R2", "Up", "Down", "Left", "Right", "X", "B", "Y", "A"};
static std::string controller_lines[4];
static int controller_row = 1;
static int rumble_total = 0;

static bool touching = false;
static int touch_x = 0, touch_y = 0;
static std::map<std::pair<int, int>, std::string> brain_text;

static bool digital_outputs[8] = {false};

std::map<std::string, std::string>& sd_files() {
  static std::map<std::string, std::string> files;
  return files;
}

MotorState& motor(int port) {
  if (port < 0 || port > 21) {
    printf("SIM: bad motor port %d\n", port);
    finish(3);
  }
  return motors[port];
}

Mode mode() { return current_mode; }

void set_mode(Mode new_mode, bool start_callback) {
  kill_key(&competition_key);
  current_mode = new_mode;
  if (new_mode == DISABLED) {
    for (MotorState& m : motors) {
      m.command = MotorState::STOPPED;
      m.stopping = vex::brakeType::coast;
    }
  }
  if (!start_callback) return;
  if (new_mode == AUTONOMOUS && auton_callback != nullptr) {
    spawn(&competition_key, "autonomous", [] { auton_callback(); });
  }
  if (new_mode == DRIVER && driver_callback != nullptr) {
    spawn(&competition_key, "usercontrol", [] { driver_callback(); });
  }
}

void boot(bool field, int (*robot_main)()) {
  static std::unique_lock<std::mutex> main_lock(mutex);
  my_lock = &main_lock;
  Thread* self = new Thread;
  self->label = "test";
  self->go = true;
  threads.push_back(self);
  current = self;
  std::thread(watchdog).detach();

  field_connected = field;
  current_mode = field ? DISABLED : DRIVER;
  spawn(&robot_main, "main", [robot_main] { robot_main(); });
}

void set_axis(int axis, double percent) { axes[axis] = percent; }

void set_button(const char* name, bool down) {
  for (int i = 0; i < 12; i++) {
    if (strcmp(button_names[i], name) == 0) {
      buttons[i] = down;
      return;
    }
  }
  printf("SIM: no button called %s\n", name);
  finish(3);
}

void tap(const char* name) {
  set_button(name, true);
  run(60);
  set_button(name, false);
  run(60);
}

void tap_screen(int x, int y) {
  touch_x = x;
  touch_y = y;
  touching = true;
  run(60);
  touching = false;
  run(60);
}

std::string controller_line(int row) { return row >= 1 && row <= 3 ? controller_lines[row] : std::string(); }

bool brain_shows(const char* text) {
  for (auto& entry : brain_text) {
    if (entry.second.find(text) != std::string::npos) return true;
  }
  return false;
}

int rumbles() { return rumble_total; }

bool digital(int triport_id) { return digital_outputs[triport_id]; }

/*---------------------------------------------------------------------------*/
/*  Physics                                                                  */
/*---------------------------------------------------------------------------*/

static const double PI = 3.14159265358979323846;
static const double FRICTION_VOLTS = 0.4;   // below this a motor doesn't move

static double free_speed_dps(vex::gearSetting cartridge) {
  switch (cartridge) {
    case vex::gearSetting::ratio36_1: return 100 * 6.0;
    case vex::gearSetting::ratio6_1: return 600 * 6.0;
    default: return 200 * 6.0;
  }
}

static double ticks_per_rev(vex::gearSetting cartridge) {
  switch (cartridge) {
    case vex::gearSetting::ratio36_1: return 1800;
    case vex::gearSetting::ratio6_1: return 300;
    default: return 900;
  }
}

// Where the motor is trying to get to (degrees per second) and how quickly it
// responds (seconds).
static void motor_target(const MotorState& m, double& target, double& tau) {
  if (current_mode == DISABLED) {
    target = 0;
    tau = 0.15;
    return;
  }
  if (m.jammed) {
    target = 0;
    tau = 0.01;
    return;
  }
  switch (m.command) {
    case MotorState::VOLTAGE: {
      double volts = m.volts > 12 ? 12 : (m.volts < -12 ? -12 : m.volts);
      volts -= m.load_volts;
      double magnitude = fabs(volts) - FRICTION_VOLTS;
      if (magnitude < 0) magnitude = 0;
      target = (volts < 0 ? -1 : 1) * magnitude / (12 - FRICTION_VOLTS) * free_speed_dps(m.cartridge);
      tau = 0.12;
      break;
    }
    case MotorState::VELOCITY:
      target = m.velocity_pct / 100.0 * free_speed_dps(m.cartridge);
      tau = 0.06;
      break;
    default:
      target = 0;
      tau = m.stopping == vex::brakeType::coast ? 0.30 : 0.04;
      break;
  }
}

static void step_motor(MotorState& m, double dt) {
  double target, tau;
  motor_target(m, target, tau);
  m.velocity_dps += (target - m.velocity_dps) * (dt / tau < 1 ? dt / tau : 1);
  m.position_deg += m.velocity_dps * dt;
}

static bool is_drive_port(int port) {
  if (robot.kind == SIM_TANK) {
    for (int p : robot.left_ports) if (p == port) return true;
    for (int p : robot.right_ports) if (p == port) return true;
    return false;
  }
  return port == robot.lf || port == robot.rf || port == robot.lb || port == robot.rb;
}

// Inches of wheel travel per degree of motor rotation.
static double inches_per_degree() { return PI * robot.wheel_diameter * robot.wheel_ratio / 360.0; }

// Steps one wheel (or one side: several motors geared together) and returns
// its surface speed in inches per second, positive for robot-forward.
static double step_wheels(const std::vector<int>& ports, double dt) {
  if (ports.empty()) return 0;
  double target_sum = 0, tau_sum = 0, speed = 0;
  for (int port : ports) {
    double target, tau;
    motor_target(motors[port], target, tau);
    target_sum += target * robot.direction[port];
    tau_sum += tau;
  }
  double target = target_sum / ports.size();
  double tau = tau_sum / ports.size();
  speed = motors[ports[0]].velocity_dps * robot.direction[ports[0]];
  speed += (target - speed) * (dt / tau < 1 ? dt / tau : 1);
  if (robot.blocked) speed = 0;
  for (int port : ports) {
    motors[port].velocity_dps = speed * robot.direction[port];
    motors[port].position_deg += motors[port].velocity_dps * dt;
  }
  return speed * inches_per_degree();
}

static void physics_step(double dt) {
  for (int port = 0; port < 22; port++) {
    if (!is_drive_port(port)) step_motor(motors[port], dt);
  }

  double forward = 0, strafe = 0, omega = 0;   // in/s, in/s, rad/s clockwise
  if (robot.kind == SIM_TANK) {
    double left = step_wheels(robot.left_ports, dt);
    double right = step_wheels(robot.right_ports, dt);
    forward = (left + right) / 2.0;
    omega = (left - right) / robot.track_width * robot.turn_scrub;
  } else {
    double lf = step_wheels({robot.lf}, dt);
    double rf = step_wheels({robot.rf}, dt);
    double lb = step_wheels({robot.lb}, dt);
    double rb = step_wheels({robot.rb}, dt);
    forward = (lf + rf + lb + rb) / 4.0;
    strafe = (lf - rf - lb + rb) / 4.0;
    omega = (lf - rf + lb - rb) / 4.0 / robot.turn_radius;
    if (robot.kind == SIM_X_DRIVE) {
      forward *= sqrt(2.0);
      strafe *= sqrt(2.0);
    } else {
      strafe *= robot.strafe_efficiency;
    }
  }

  double heading = robot.heading_deg * PI / 180.0;
  robot.x += (forward * sin(heading) + strafe * cos(heading)) * dt;
  robot.y += (forward * cos(heading) - strafe * sin(heading)) * dt;
  robot.heading_deg += omega * dt * 180.0 / PI;
  robot.forward_speed = forward;
  robot.strafe_speed = strafe;

  tracker_forward_in += (forward - omega * robot.forward_tracker_offset) * dt;
  tracker_sideways_in += (strafe - omega * robot.sideways_tracker_offset) * dt;
}

}  // namespace sim

/*---------------------------------------------------------------------------*/
/*  The fake VEX API                                                         */
/*---------------------------------------------------------------------------*/

int32_t vexDeviceGetStatus(V5_DeviceType* buffer) {
  for (int i = 0; i < V5_MAX_DEVICE_PORTS; i++) buffer[i] = sim::devices[i];
  return V5_MAX_DEVICE_PORTS;
}

// A device handle is just its port number plus one, so it is never null.
V5_DeviceT vexDeviceGetByIndex(uint32_t index) { return (V5_DeviceT)(uintptr_t)(index + 1); }

double vexDeviceMotorTemperatureGet(V5_DeviceT device) {
  return sim::motors[(uintptr_t)device - 1].temperature;
}

namespace vex {

using sim::MotorState;

const color color::black = color(0x000000);
const color color::white = color(0xFFFFFF);

uint32_t timer::system() { return (uint32_t)sim::now_ms; }

// Like the real Brain, a task is identified by its function: starting one
// that is already running restarts it.
task::task(int (*callback)(void)) : _callback(callback) {
  const void* key = reinterpret_cast<const void*>(callback);
  sim::kill_key(key);
  sim::spawn(key, "task", [callback] { callback(); });
}

void task::stop() {
  if (_callback != nullptr) sim::kill_key(reinterpret_cast<const void*>(_callback));
}

void task::sleep(uint32_t time) { sim::sleep_ms(time); }

void task::yield() { sim::sleep_ms(0); }

void wait(double time, timeUnits units) { sim::sleep_ms(units == timeUnits::sec ? time * 1000.0 : time); }

// ---- motor ----

double motor::sign(directionType dir) const {
  return (_reversed ? -1.0 : 1.0) * (dir == directionType::rev ? -1.0 : 1.0);
}

bool motor::installed() { return true; }

void motor::spin(directionType dir, double value, voltageUnits units) {
  if (sim::current_mode == sim::DISABLED) return;
  MotorState& m = sim::motor(_index);
  m.command = MotorState::VOLTAGE;
  m.volts = sign(dir) * (units == voltageUnits::mV ? value / 1000.0 : value);
}

void motor::spin(directionType dir, double value, velocityUnits units) {
  if (sim::current_mode == sim::DISABLED) return;
  MotorState& m = sim::motor(_index);
  double percent = value;
  if (units == velocityUnits::rpm) percent = value / (sim::free_speed_dps(_gears) / 6.0) * 100.0;
  if (units == velocityUnits::dps) percent = value / sim::free_speed_dps(_gears) * 100.0;
  m.command = MotorState::VELOCITY;
  m.velocity_pct = sign(dir) * percent;
}

void motor::stop() { sim::motor(_index).command = MotorState::STOPPED; }

void motor::stop(brakeType mode) {
  MotorState& m = sim::motor(_index);
  m.command = MotorState::STOPPED;
  m.stopping = mode;
}

void motor::setStopping(brakeType mode) { sim::motor(_index).stopping = mode; }

void motor::setMaxTorque(double, currentUnits) {}

// The motor reports degrees based on the cartridge the code says it has. If
// that is not the cartridge really fitted, the reading is scaled wrong, just
// like on a real robot.
static double reading_scale(MotorState& m, gearSetting assumed) {
  return sim::ticks_per_rev(m.cartridge) / sim::ticks_per_rev(assumed);
}

static double motor_zero[22] = {0};

void motor::setPosition(double value, rotationUnits units) {
  MotorState& m = sim::motor(_index);
  double degrees_wanted = units == rotationUnits::rev ? value * 360.0 : value;
  double raw = (_reversed ? -1.0 : 1.0) * m.position_deg * reading_scale(m, _gears);
  motor_zero[_index] = raw - degrees_wanted;
}

double motor::position(rotationUnits units) {
  MotorState& m = sim::motor(_index);
  double raw = (_reversed ? -1.0 : 1.0) * m.position_deg * reading_scale(m, _gears);
  double degrees_now = raw - motor_zero[_index];
  return units == rotationUnits::rev ? degrees_now / 360.0 : degrees_now;
}

double motor::velocity(velocityUnits units) {
  MotorState& m = sim::motor(_index);
  double percent = (_reversed ? -1.0 : 1.0) * m.velocity_dps / sim::free_speed_dps(m.cartridge) * 100.0;
  if (units == velocityUnits::rpm) return percent / 100.0 * sim::free_speed_dps(_gears) / 6.0;
  if (units == velocityUnits::dps) return percent / 100.0 * sim::free_speed_dps(_gears);
  return percent;
}

double motor::voltage(voltageUnits units) {
  MotorState& m = sim::motor(_index);
  double volts = m.command == MotorState::VOLTAGE && sim::current_mode != sim::DISABLED ? m.volts : 0;
  volts *= _reversed ? -1.0 : 1.0;
  return units == voltageUnits::mV ? volts * 1000.0 : volts;
}

double motor::temperature(temperatureUnits) { return sim::motor(_index).temperature; }

// ---- sensors ----

bool inertial::installed() { return sim::imu_installed; }

void inertial::calibrate() { sim::imu_calibrating_until = sim::now_ms + 2000; }

bool inertial::isCalibrating() { return sim::now_ms < sim::imu_calibrating_until; }

double inertial::rotation(rotationUnits) { return sim::robot.heading_deg + sim::imu_offset_deg; }

double inertial::heading(rotationUnits) {
  double value = fmod(sim::robot.heading_deg + sim::imu_offset_deg, 360.0);
  return value < 0 ? value + 360.0 : value;
}

void inertial::setRotation(double value, rotationUnits) { sim::imu_offset_deg = value - sim::robot.heading_deg; }

double rotation::position(rotationUnits units) {
  double inches = 0;
  if (_index == sim::robot.forward_tracker_port) inches = sim::tracker_forward_in - sim::tracker_forward_zero;
  if (_index == sim::robot.sideways_tracker_port) inches = sim::tracker_sideways_in - sim::tracker_sideways_zero;
  double degrees_now = inches / (sim::PI * sim::robot.tracker_diameter) * 360.0 * (_reversed ? -1.0 : 1.0);
  return units == rotationUnits::rev ? degrees_now / 360.0 : degrees_now;
}

void rotation::setPosition(double, rotationUnits) {
  if (_index == sim::robot.forward_tracker_port) sim::tracker_forward_zero = sim::tracker_forward_in;
  if (_index == sim::robot.sideways_tracker_port) sim::tracker_sideways_zero = sim::tracker_sideways_in;
}

void digital_out::set(bool value) { sim::digital_outputs[_id] = value; }

int32_t digital_out::value() { return sim::digital_outputs[_id] ? 1 : 0; }

// ---- brain ----

void brain::lcd::clearScreen() {
  sim::brain_text.clear();
  sim::brain_line_total = 0;
}

void brain::lcd::drawLine(int x1, int y1, int x2, int y2) {
  sim::brain_line_total++;
  // The Brain screen is 480 x 240. Drawing outside it means a scaling bug.
  if (x1 < 0 || x1 > 479 || x2 < 0 || x2 > 479 || y1 < 0 || y1 > 239 || y2 < 0 || y2 > 239) {
    sim::brain_line_off_screen = true;
  }
}

void brain::lcd::setPenWidth(uint32_t) {}

double optical::hue() { return sim::optical_hue; }
bool optical::isNearObject() { return sim::optical_near; }
void optical::setLight(ledState state) { sim::optical_light = state == ledState::on; }
void optical::setLightPower(int32_t, percentUnits) {}
void brain::lcd::setFont(fontType) {}
void brain::lcd::setFillColor(const color&) {}
void brain::lcd::setPenColor(const color&) {}
void brain::lcd::drawRectangle(int, int, int, int) {}

void brain::lcd::printAt(int32_t x, int32_t y, const char* format, ...) {
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  sim::brain_text[{x, y}] = buffer;
}

void brain::lcd::printAt(int32_t x, int32_t y, bool, const char* format, ...) {
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  sim::brain_text[{x, y}] = buffer;
}

bool brain::lcd::pressing() { return sim::touching; }
int32_t brain::lcd::xPosition() { return sim::touch_x; }
int32_t brain::lcd::yPosition() { return sim::touch_y; }

uint32_t brain::battery::capacity(percentUnits) { return 87; }

bool brain::sdcard::isInserted() { return sim::sd_inserted; }

bool brain::sdcard::exists(const char* name) { return sim::sd_inserted && sim::sd_files().count(name) > 0; }

int32_t brain::sdcard::loadfile(const char* name, uint8_t* buffer, int32_t len) {
  if (!exists(name)) return 0;
  const std::string& data = sim::sd_files()[name];
  int32_t n = (int32_t)data.size() < len ? (int32_t)data.size() : len;
  memcpy(buffer, data.data(), n);
  return n;
}

int32_t brain::sdcard::savefile(const char* name, uint8_t* buffer, int32_t len) {
  if (!sim::sd_inserted) return 0;
  sim::sd_files()[name] = std::string((const char*)buffer, len);
  return len;
}

// ---- controller ----

bool controller::button::pressing() const { return sim::buttons[_id]; }

int32_t controller::axis::position(percentUnits) const { return (int32_t)sim::axes[_id]; }

int32_t controller::axis::value() const { return (int32_t)(sim::axes[_id] * 1.27); }

void controller::lcd::setCursor(int32_t row, int32_t) {
  sim::controller_row = row >= 1 && row <= 3 ? row : 1;
}

void controller::lcd::print(const char* format, ...) {
  char buffer[128];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  sim::controller_lines[sim::controller_row] = buffer;
}

void controller::lcd::clearScreen() {
  for (std::string& line : sim::controller_lines) line.clear();
}

void controller::lcd::clearLine(int number) {
  if (number >= 1 && number <= 3) sim::controller_lines[number].clear();
}

void controller::lcd::clearLine() { sim::controller_lines[sim::controller_row].clear(); }

void controller::rumble(const char*) { sim::rumble_total++; }

// ---- competition ----

void competition::autonomous(void (*callback)(void)) {
  sim::auton_callback = callback;
  if (sim::current_mode == sim::AUTONOMOUS) sim::set_mode(sim::AUTONOMOUS);
}

void competition::drivercontrol(void (*callback)(void)) {
  sim::driver_callback = callback;
  if (sim::current_mode == sim::DRIVER) sim::set_mode(sim::DRIVER);
}

bool competition::isEnabled() { return sim::current_mode != sim::DISABLED; }
bool competition::isDriverControl() { return sim::current_mode != sim::AUTONOMOUS; }
bool competition::isAutonomous() { return sim::current_mode == sim::AUTONOMOUS; }
bool competition::isCompetitionSwitch() { return sim::field_connected; }
bool competition::isFieldControl() { return false; }

}  // namespace vex

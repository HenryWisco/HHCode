// Controls for the simulated robot. Tests use these to play the part of the
// field, the driver and the laws of physics.
#pragma once
#include <functional>
#include <map>
#include <string>
#include "v5.h"

namespace sim {

enum Mode { DISABLED, AUTONOMOUS, DRIVER };

/** One smart motor, as it physically is (not how the code thinks of it). */
struct MotorState {
  enum Command { STOPPED, VOLTAGE, VELOCITY };
  Command command = STOPPED;
  double volts = 0;            // when command == VOLTAGE
  double velocity_pct = 0;     // when command == VELOCITY
  vex::brakeType stopping = vex::brakeType::coast;
  vex::gearSetting cartridge = vex::gearSetting::ratio18_1;   // the one really fitted
  double position_deg = 0;     // of the motor shaft
  double velocity_dps = 0;
  double temperature = 30;
  bool jammed = false;         // can't turn
  double load_volts = 0;       // constant load, like gravity on a lift
};

enum DriveKind { SIM_TANK, SIM_MECANUM, SIM_X_DRIVE };

/** A distance sensor bolted to the robot. */
struct DistanceSensor {
  int port;
  double forward, right;   // inches from the robot's center
  double facing_deg;       // clockwise from straight ahead
};

/** The physical robot. Filled in by describe_robot() in robot.cpp. */
struct Robot {
  DriveKind kind = SIM_TANK;
  std::vector<int> left_ports, right_ports;   // tank
  int lf = -1, rf = -1, lb = -1, rb = -1;     // mecanum / X-drive
  double direction[22];        // +1, or -1 for motors mounted mirrored
  double wheel_diameter = 3.25;
  double wheel_ratio = 1;      // wheel turns per motor turn
  double track_width = 12;     // tank: inches between the left and right wheels
  double turn_radius = 8;      // holonomic: inches from the center to each wheel
  double turn_scrub = 0.6;     // tank: how much of the ideal turning speed survives wheel scrub
  double strafe_efficiency = 1;   // mecanum: sideways inches moved per inch the wheels roll
  int forward_tracker_port = -1;
  int sideways_tracker_port = -1;
  double forward_tracker_offset = 0;    // inches to the right of center
  double sideways_tracker_offset = 0;   // inches behind center
  double tracker_diameter = 2.75;
  bool blocked = false;        // pinned against a wall: the drive wheels can't turn
  std::vector<DistanceSensor> distance_sensors;

  // True position on the field. Heading is clockwise from +Y, like JAR-Template.
  double x = 0, y = 0, heading_deg = 0;
  double forward_speed = 0, strafe_speed = 0;   // inches per second
  Robot() { for (int i = 0; i < 22; i++) direction[i] = 1; }
};

extern Robot robot;
extern bool sd_inserted;
// The field walls, in the same coordinates as robot.x and robot.y. The robot
// starts at (0, 0), so by default it starts in the middle of the field.
extern double field_size;
extern double field_center_x, field_center_y;
/**
 * Something standing in front of a distance sensor: the sensor on `port`
 * reads `inches` instead of the distance to the wall. A negative number
 * takes the object away again.
 */
void block_distance(int port, double inches);
extern bool imu_installed;
// The optical sensor: what is in front of it, and whether its light is on.
extern double optical_hue;
extern bool optical_near;
extern bool optical_light;
/** What is plugged into each smart port (index 0 is port 1). Unplug one by setting it to kDeviceTypeNoSensor. */
extern V5_DeviceType devices[V5_MAX_DEVICE_PORTS];
/** How many line segments have been drawn on the Brain since it was last cleared. */
int brain_lines();
std::map<std::string, std::string>& sd_files();
MotorState& motor(int port);

/** Starts the scheduler and the robot program. `field` = plugged into a competition switch. */
void boot(bool field, int (*robot_main)());
/** Lets `ms` milliseconds of robot time pass. */
void run(double ms);
double now();
/** Changes the match period. With start_callback false, autonomous()/usercontrol() are not started. */
void set_mode(Mode mode, bool start_callback = true);
Mode mode();

void set_axis(int axis, double percent);
void set_button(const char* name, bool down);
/** Presses a button for 60 ms, then waits another 60 ms. */
void tap(const char* name);
void tap_screen(int x, int y);

std::string controller_line(int row);
bool brain_shows(const char* text);
int rumbles();
bool digital(int triport_id);

/** Ends the program right away with this exit code. */
void finish(int code);

}  // namespace sim

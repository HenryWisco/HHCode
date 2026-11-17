#include "vex.h"

/**
 * Drive constructor for the chassis.
 * Even though there's only one constructor, there can be
 * huge differences in implementation depending on the drive style
 * selected.
 * 
 * @param drive_setup The style of drive, such as TANK_TWO_ROTATION.
 * @param DriveL Left motor group.
 * @param DriveR Right motor group.
 * @param gyro_port IMU port.
 * @param wheel_diameter Wheel diameter in inches.
 * @param wheel_ratio External drive gear ratio.
 * @param gyro_scale Scale factor in degrees.
 * @param DriveLF_port Left front port for holonomic drives.
 * @param DriveRF_port Right front port for holonomic drives.
 * @param DriveLB_port Left back port for holonomic drives.
 * @param DriveRB_port Right back port for holonomic drives.
 * @param ForwardTracker_port Port for the forward tracker.
 * @param ForwardTracker_diameter Diameter in inches.
 * @param ForwardTracker_center_distance Horizontal distance in inches.
 * @param SidewaysTracker_port Port for the sideways tracker.
 * @param SidewaysTracker_diameter Diameter in inches.
 * @param SidewaysTracker_center_distance Vertical distance in inches.
 */

Drive::Drive(enum::drive_setup drive_setup, motor_group DriveL, motor_group DriveR, 
int gyro_port, float wheel_diameter, float wheel_ratio, float gyro_scale, 
int DriveLF_port, int DriveRF_port, int DriveLB_port, int DriveRB_port, 
int ForwardTracker_port, float ForwardTracker_diameter, float ForwardTracker_center_distance, 
int SidewaysTracker_port, float SidewaysTracker_diameter, float SidewaysTracker_center_distance) :
  wheel_diameter(wheel_diameter),
  wheel_ratio(wheel_ratio),
  gyro_scale(gyro_scale),
  drive_in_to_deg_ratio(wheel_ratio/360.0*M_PI*wheel_diameter),
  ForwardTracker_center_distance(ForwardTracker_center_distance),
  ForwardTracker_diameter(ForwardTracker_diameter),
  ForwardTracker_in_to_deg_ratio(M_PI*ForwardTracker_diameter/360.0),
  SidewaysTracker_center_distance(SidewaysTracker_center_distance),
  SidewaysTracker_diameter(SidewaysTracker_diameter),
  SidewaysTracker_in_to_deg_ratio(M_PI*SidewaysTracker_diameter/360.0),
  drive_setup(drive_setup),
  DriveL(DriveL),
  DriveR(DriveR),
  Gyro(inertial(gyro_port)),
  DriveLF(abs(DriveLF_port), is_reversed(DriveLF_port)),
  DriveRF(abs(DriveRF_port), is_reversed(DriveRF_port)),
  DriveLB(abs(DriveLB_port), is_reversed(DriveLB_port)),
  DriveRB(abs(DriveRB_port), is_reversed(DriveRB_port)),
  R_ForwardTracker(ForwardTracker_port),
  R_SidewaysTracker(SidewaysTracker_port),
  E_ForwardTracker(ThreeWire.Port[to_port(ForwardTracker_port)]),
  E_SidewaysTracker(ThreeWire.Port[to_port(SidewaysTracker_port)])
{
    if (drive_setup == TANK_ONE_FORWARD_ENCODER || drive_setup == TANK_ONE_FORWARD_ROTATION || drive_setup == ZERO_TRACKER_ODOM){
      odom.set_physical_distances(ForwardTracker_center_distance, 0);
    } 
    if (drive_setup == TANK_ONE_SIDEWAYS_ENCODER || drive_setup == TANK_ONE_SIDEWAYS_ROTATION || 
    drive_setup == TANK_TWO_ENCODER || drive_setup == TANK_TWO_ROTATION ||
    drive_setup == HOLONOMIC_TWO_ENCODER || drive_setup == HOLONOMIC_TWO_ROTATION){
      odom.set_physical_distances(ForwardTracker_center_distance, SidewaysTracker_center_distance);
    }
    // HHCode: encoder-only holonomic odom measures from the center of the robot.
    if (drive_setup == HOLONOMIC_ZERO_TRACKER_ODOM){
      odom.set_physical_distances(0, 0);
    }
    // HHCode: remembered so set_holonomic_cartridge() can rebuild the motors.
    holonomic_ports[0] = DriveLF_port;
    holonomic_ports[1] = DriveRF_port;
    holonomic_ports[2] = DriveLB_port;
    holonomic_ports[3] = DriveRB_port;
}

/**
 * Drives each side of the chassis at the specified voltage.
 * 
 * @param leftVoltage Voltage out of 12.
 * @param rightVoltage Voltage out of 12.
 */

void Drive::drive_with_voltage(float leftVoltage, float rightVoltage){
  // HHCode: a mirrored auton swaps the two sides.
  if (mirrored) { float swap = leftVoltage; leftVoltage = rightVoltage; rightVoltage = swap; }
  DriveL.spin(fwd, leftVoltage, volt);
  DriveR.spin(fwd, rightVoltage,volt);
}

/**
 * Resets default turn constants.
 * Turning includes turn_to_angle() and turn_to_point().
 * 
 * @param turn_max_voltage Max voltage out of 12.
 * @param turn_kp Proportional constant.
 * @param turn_ki Integral constant.
 * @param turn_kd Derivative constant.
 * @param turn_starti Minimum angle in degrees for integral to begin.
 */

void Drive::set_turn_constants(float turn_max_voltage, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  this->turn_max_voltage = turn_max_voltage;
  this->turn_kp = turn_kp;
  this->turn_ki = turn_ki;
  this->turn_kd = turn_kd;
  this->turn_starti = turn_starti;
} 

/**
 * Resets default drive constants.
 * Driving includes drive_distance(), drive_to_point(), and
 * holonomic_drive_to_point().
 * 
 * @param drive_max_voltage Max voltage out of 12.
 * @param drive_kp Proportional constant.
 * @param drive_ki Integral constant.
 * @param drive_kd Derivative constant.
 * @param drive_starti Minimum distance in inches for integral to begin
 */

void Drive::set_drive_constants(float drive_max_voltage, float drive_kp, float drive_ki, float drive_kd, float drive_starti){
  this->drive_max_voltage = drive_max_voltage;
  this->drive_kp = drive_kp;
  this->drive_ki = drive_ki;
  this->drive_kd = drive_kd;
  this->drive_starti = drive_starti;
} 

/**
 * Resets default heading constants.
 * Heading control keeps the robot facing the right direction
 * and is part of drive_distance(), drive_to_point(), and
 * holonomic_drive_to_point.
 * 
 * @param heading_max_voltage Max voltage out of 12.
 * @param heading_kp Proportional constant.
 * @param heading_ki Integral constant.
 * @param heading_kd Derivative constant.
 * @param heading_starti Minimum angle in degrees for integral to begin.
 */

void Drive::set_heading_constants(float heading_max_voltage, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  this->heading_max_voltage = heading_max_voltage;
  this->heading_kp = heading_kp;
  this->heading_ki = heading_ki;
  this->heading_kd = heading_kd;
  this->heading_starti = heading_starti;
}

/**
 * Resets default swing constants.
 * Swing control holds one side of the drive still and turns with the other.
 * Only left_swing_to_angle() and right_swing_to_angle() use these constants.
 * 
 * @param swing_max_voltage Max voltage out of 12.
 * @param swing_kp Proportional constant.
 * @param swing_ki Integral constant.
 * @param swing_kd Derivative constant.
 * @param swing_starti Minimum angle in degrees for integral to begin.
 */

void Drive::set_swing_constants(float swing_max_voltage, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  this->swing_max_voltage = swing_max_voltage;
  this->swing_kp = swing_kp;
  this->swing_ki = swing_ki;
  this->swing_kd = swing_kd;
  this->swing_starti = swing_starti;
} 

/**
 * Resets default turn exit conditions.
 * The robot exits when error is less than settle_error for a duration of settle_time, 
 * or if the function has gone on for longer than timeout.
 * 
 * @param turn_settle_error Error to be considered settled in degrees.
 * @param turn_settle_time Time to be considered settled in milliseconds.
 * @param turn_timeout Time before quitting and move on in milliseconds.
 */

void Drive::set_turn_exit_conditions(float turn_settle_error, float turn_settle_time, float turn_timeout){
  this->turn_settle_error = turn_settle_error;
  this->turn_settle_time = turn_settle_time;
  this->turn_timeout = turn_timeout;
}

/**
 * Resets default drive exit conditions.
 * The robot exits when error is less than settle_error for a duration of settle_time, 
 * or if the function has gone on for longer than timeout.
 * 
 * @param drive_settle_error Error to be considered settled in inches.
 * @param drive_settle_time Time to be considered settled in milliseconds.
 * @param drive_timeout Time before quitting and move on in milliseconds.
 */

void Drive::set_drive_exit_conditions(float drive_settle_error, float drive_settle_time, float drive_timeout){
  this->drive_settle_error = drive_settle_error;
  this->drive_settle_time = drive_settle_time;
  this->drive_timeout = drive_timeout;
}

/**
 * Resets default swing exit conditions.
 * The robot exits when error is less than settle_error for a duration of settle_time, 
 * or if the function has gone on for longer than timeout.
 * 
 * @param swing_settle_error Error to be considered settled in degrees.
 * @param swing_settle_time Time to be considered settled in milliseconds.
 * @param swing_timeout Time before quitting and move on in milliseconds.
 */

void Drive::set_swing_exit_conditions(float swing_settle_error, float swing_settle_time, float swing_timeout){
  this->swing_settle_error = swing_settle_error;
  this->swing_settle_time = swing_settle_time;
  this->swing_timeout = swing_timeout;
}

/**
 * Gives the drive's absolute heading with Gyro correction.
 * 
 * @return Gyro scale-corrected heading in the range [0, 360).
 */

float Drive::get_absolute_heading(){ 
  return( reduce_0_to_360( get_rotation() ) ); 
}

/**
 * Gets the motor group's position and converts to inches.
 * 
 * @return Left position in inches.
 */

// HHCode: X-drive wheels sit at 45 degrees, so the robot travels sqrt(2) times
// farther than the wheels roll.
static float x_drive_factor(){
  return( hh::driver::drive_type == hh::X_DRIVE ? M_SQRT2 : 1.0 );
}

float Drive::get_left_position_in(){
  return( (mirrored ? DriveR : DriveL).position(deg)*drive_in_to_deg_ratio*x_drive_factor() );
}

/**
 * Gets the motor group's position and converts to inches.
 * 
 * @return Right position in inches.
 */

float Drive::get_right_position_in(){
  return( (mirrored ? DriveL : DriveR).position(deg)*drive_in_to_deg_ratio*x_drive_factor() );
}

/**
 * Stops both sides of the drive with the desired mode.
 * 
 * @param mode hold, brake, or stop
 */

void Drive::drive_stop(vex::brakeType mode){
  DriveL.stop(mode);
  DriveR.stop(mode);
}

/**
 * Turns the robot to a field-centric angle.
 * Optimizes direction, so it turns whichever way is closer to the 
 * current heading of the robot.
 * 
 * @param angle Desired angle in degrees.
 */

void Drive::turn_to_angle(float angle){
  turn_to_angle(angle, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_angle(float angle, float turn_max_voltage){
  turn_to_angle(angle, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_angle(float angle, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout){
  turn_to_angle(angle, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_angle(float angle, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  // HHCode: async, chain, slew and stall support.
  if (begin_motion(MOTION_TURN, [=]{ turn_to_angle(angle, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti); })) { return; }
  if (take_chain()) { turn_settle_error = turn_chain_error; turn_settle_time = 0; }
  float previous_output = current_turn_voltage();

  // HHCode: clockwise() / counterclockwise() force the long way round when asked.
  int direction = take_turn_direction();
  float start_error = reduce_negative_180_to_180(angle - get_absolute_heading());
  if (fabs(start_error) <= turn_settle_error) { direction = 0; }
  if (direction > 0 && start_error < 0) { start_error += 360; }
  if (direction < 0 && start_error > 0) { start_error -= 360; }
  float target_rotation = get_rotation() + start_error;

  PID turnPID(start_error, turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
  while( !turnPID.is_settled() ){
    float error = direction == 0 ? reduce_negative_180_to_180(angle - get_absolute_heading()) : target_rotation - get_rotation();
    float output = turnPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    output = slew(output, previous_output, turn_slew);
    previous_output = output;
    if (check_stall(output)) { break; }
    drive_with_voltage(output, -output);
    hh::graph::record(error, output);   // HHCode
    task::sleep(10);
  }
}

/**
 * Drives the robot a given distance with a given heading.
 * Drive distance does not optimize for direction, so it won't try
 * to drive at the opposite heading from the one given to get there faster.
 * You can control the heading, but if you choose not to, it will drive with the
 * heading it's currently facing. It uses the average of the left and right
 * motor groups to calculate distance driven.
 * 
 * @param distance Desired distance in inches.
 * @param heading Desired heading in degrees.
 */

void Drive::drive_distance(float distance){
  drive_distance(distance, get_absolute_heading());
}

void Drive::drive_distance(float distance, float heading){
  // HHCode: backward drives can have their own constants.
  if (distance < 0 && use_backward_constants){
    drive_distance(distance, heading, backward_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, backward_kp, backward_ki, backward_kd, backward_starti, heading_kp, heading_ki, heading_kd, heading_starti);
    return;
  }
  drive_distance(distance, heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage){
  drive_distance(distance, heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  drive_distance(distance, heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  // HHCode: async, chain, slew and stall support.
  if (begin_motion(MOTION_DRIVE, [=]{ drive_distance(distance, heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti); })) { return; }
  if (take_chain()) { drive_settle_error = drive_chain_error; drive_settle_time = 0; }
  float previous_output = current_drive_voltage();

  PID drivePID(distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(reduce_negative_180_to_180(heading - get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  float start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
  float average_position = start_average_position;
  while(drivePID.is_settled() == false){
    average_position = (get_left_position_in()+get_right_position_in())/2.0;
    float drive_error = distance+start_average_position-average_position;
    float heading_error = reduce_negative_180_to_180(heading - get_absolute_heading());
    float drive_output = drivePID.compute(drive_error);
    float heading_output = headingPID.compute(heading_error);

    drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    drive_output = slew(drive_output, previous_output, drive_slew);
    previous_output = drive_output;
    if (check_stall(drive_output)) { break; }

    drive_with_voltage(drive_output+heading_output, drive_output-heading_output);
    hh::graph::record(drive_error, drive_output);   // HHCode
    task::sleep(10);
  }
}

/**
 * Turns to a given angle with only one side of the drivetrain.
 * Like turn_to_angle(), is optimized for turning the shorter
 * direction.
 * 
 * @param angle Desired angle in degrees.
 */

void Drive::left_swing_to_angle(float angle){
  left_swing_to_angle(angle, swing_max_voltage, swing_settle_error, swing_settle_time, swing_timeout, swing_kp, swing_ki, swing_kd, swing_starti);
}

void Drive::left_swing_to_angle(float angle, float swing_max_voltage, float swing_settle_error, float swing_settle_time, float swing_timeout, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  // HHCode: async and chain support.
  if (begin_motion(MOTION_TURN, [=]{ left_swing_to_angle(angle, swing_max_voltage, swing_settle_error, swing_settle_time, swing_timeout, swing_kp, swing_ki, swing_kd, swing_starti); })) { return; }
  if (take_chain()) { swing_settle_error = turn_chain_error; swing_settle_time = 0; }

  PID swingPID(reduce_negative_180_to_180(angle - get_absolute_heading()), swing_kp, swing_ki, swing_kd, swing_starti, swing_settle_error, swing_settle_time, swing_timeout);
  while(swingPID.is_settled() == false){
    float error = reduce_negative_180_to_180(angle - get_absolute_heading());
    float output = swingPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    (mirrored ? DriveR : DriveL).spin(fwd, output, volt);
    (mirrored ? DriveL : DriveR).stop(hold);
    hh::graph::record(error, output);   // HHCode
    task::sleep(10);
  }
}

void Drive::right_swing_to_angle(float angle){
  right_swing_to_angle(angle, swing_max_voltage, swing_settle_error, swing_settle_time, swing_timeout, swing_kp, swing_ki, swing_kd, swing_starti);
}

void Drive::right_swing_to_angle(float angle, float swing_max_voltage, float swing_settle_error, float swing_settle_time, float swing_timeout, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  // HHCode: async and chain support.
  if (begin_motion(MOTION_TURN, [=]{ right_swing_to_angle(angle, swing_max_voltage, swing_settle_error, swing_settle_time, swing_timeout, swing_kp, swing_ki, swing_kd, swing_starti); })) { return; }
  if (take_chain()) { swing_settle_error = turn_chain_error; swing_settle_time = 0; }

  PID swingPID(reduce_negative_180_to_180(angle - get_absolute_heading()), swing_kp, swing_ki, swing_kd, swing_starti, swing_settle_error, swing_settle_time, swing_timeout);
  while(swingPID.is_settled() == false){
    float error = reduce_negative_180_to_180(angle - get_absolute_heading());
    float output = swingPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    (mirrored ? DriveL : DriveR).spin(reverse, output, volt);
    (mirrored ? DriveR : DriveL).stop(hold);
    hh::graph::record(error, output);   // HHCode
    task::sleep(10);
  }
}

/**
 * Depending on the drive style, gets the tracker's position.
 * 
 * @return The tracker position.
 */

float Drive::get_ForwardTracker_position(){
  // HHCode: encoder-only holonomic odom uses the average of both sides.
  if (drive_setup==HOLONOMIC_ZERO_TRACKER_ODOM){
    return((get_left_position_in()+get_right_position_in())/2.0);
  }
  if (drive_setup==ZERO_TRACKER_ODOM || drive_setup == TANK_ONE_SIDEWAYS_ENCODER || drive_setup == TANK_ONE_SIDEWAYS_ROTATION){
    return(get_right_position_in());
  }
  if (drive_setup==TANK_ONE_FORWARD_ENCODER || drive_setup == TANK_TWO_ENCODER || drive_setup == HOLONOMIC_TWO_ENCODER){
    return(E_ForwardTracker.position(deg)*ForwardTracker_in_to_deg_ratio);
  }else{
    return(R_ForwardTracker.position(deg)*ForwardTracker_in_to_deg_ratio);
  }
}

/**
 * Depending on the drive style, gets the tracker's position.
 * 
 * @return The tracker position.
 */

float Drive::get_SidewaysTracker_position(){
  // HHCode: encoder-only holonomic odom measures strafing from the wheels.
  if (drive_setup==HOLONOMIC_ZERO_TRACKER_ODOM){
    return(get_strafe_position_in());
  }
  if (drive_setup==TANK_ONE_FORWARD_ENCODER || drive_setup == TANK_ONE_FORWARD_ROTATION || drive_setup == ZERO_TRACKER_ODOM){
    return(0);
  }else if (drive_setup == TANK_TWO_ENCODER || drive_setup == HOLONOMIC_TWO_ENCODER || drive_setup == TANK_ONE_SIDEWAYS_ENCODER){
    return((mirrored ? -1.0 : 1.0)*E_SidewaysTracker.position(deg)*SidewaysTracker_in_to_deg_ratio);
  }else{
    return((mirrored ? -1.0 : 1.0)*R_SidewaysTracker.position(deg)*SidewaysTracker_in_to_deg_ratio);
  }
}

/**
 * Background task for updating the odometry.
 */

void Drive::position_track(){
  while(1){
    odom.update_position(get_ForwardTracker_position(), get_SidewaysTracker_position(), get_absolute_heading());
    task::sleep(5);
  }
}

/**
 * Resets the robot's heading.
 * For example, at the beginning of auton, if your robot starts at
 * 45 degrees, so set_heading(45) and the robot will know which way 
 * it's facing.
 * 
 * @param orientation_deg Desired heading in degrees.
 */

void Drive::set_heading(float orientation_deg){
  Gyro.setRotation((mirrored ? -1.0 : 1.0)*orientation_deg*gyro_scale/360.0, deg);
}

/**
 * Resets the robot's coordinates and heading.
 * This is for odom-using robots to specify where the bot is at the beginning
 * of the match.
 * 
 * @param X_position Robot's x in inches.
 * @param Y_position Robot's y in inches.
 * @param orientation_deg Desired heading in degrees.
 */

void Drive::set_coordinates(float X_position, float Y_position, float orientation_deg){
  odom.set_position(X_position, Y_position, orientation_deg, get_ForwardTracker_position(), get_SidewaysTracker_position());
  set_heading(orientation_deg);
  odom_task = task(position_track_task);
}

/**
 * Gets the robot's x.
 * 
 * @return The robot's x position in inches.
 */

float Drive::get_X_position(){
  return(odom.X_position);
}

/**
 * Gets the robot's y.
 * 
 * @return The robot's y position in inches.
 */

float Drive::get_Y_position(){
  return(odom.Y_position);
}

/**
 * Drives to a specified point on the field.
 * Uses the double-PID method, with one for driving and one for heading correction.
 * The drive error is the euclidean distance to the desired point, and the heading error
 * is the turn correction from the current heading to the desired point. Uses optimizations
 * like driving backwards whenever possible and scaling the drive output with the cosine
 * of the angle to the point.
 * 
 * @param X_position Desired x position in inches.
 * @param Y_position Desired y position in inches.
 */

void Drive::drive_to_point(float X_position, float Y_position){
  drive_to_point(X_position, Y_position, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(float X_position, float Y_position, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage){
  drive_to_point(X_position, Y_position, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(float X_position, float Y_position, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  drive_to_point(X_position, Y_position, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(float X_position, float Y_position, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  // HHCode: async, chain, slew and stall support.
  if (begin_motion(MOTION_ODOM, [=]{ drive_to_point(X_position, Y_position, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti); })) { return; }
  if (take_chain()) { drive_settle_error = drive_chain_error; drive_settle_time = 0; }
  float previous_output = current_drive_voltage();

  PID drivePID(hypot(X_position-get_X_position(),Y_position-get_Y_position()), drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  float start_angle_deg = to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()));
  PID headingPID(start_angle_deg-get_absolute_heading(), heading_kp, heading_ki, heading_kd, heading_starti);
  bool line_settled = false;
  bool prev_line_settled = is_line_settled(X_position, Y_position, start_angle_deg, get_X_position(), get_Y_position());
  while(!drivePID.is_settled()){
    line_settled = is_line_settled(X_position, Y_position, start_angle_deg, get_X_position(), get_Y_position());
    if(line_settled && !prev_line_settled){ break; }
    prev_line_settled = line_settled;

    float drive_error = hypot(X_position-get_X_position(),Y_position-get_Y_position());
    float heading_error = reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading());
    float drive_output = drivePID.compute(drive_error);

    float heading_scale_factor = cos(to_rad(heading_error));
    drive_output*=heading_scale_factor;
    heading_error = reduce_negative_90_to_90(heading_error);
    float heading_output = headingPID.compute(heading_error);
    
    if (drive_error<drive_settle_error) { heading_output = 0; }

    drive_output = clamp(drive_output, -fabs(heading_scale_factor)*drive_max_voltage, fabs(heading_scale_factor)*drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    drive_output = slew(drive_output, previous_output, drive_slew);
    previous_output = drive_output;
    if (check_stall(drive_output)) { break; }

    drive_output = clamp_min_voltage(drive_output, drive_min_voltage);

    drive_with_voltage(left_voltage_scaling(drive_output, heading_output), right_voltage_scaling(drive_output, heading_output));
    hh::graph::record(drive_error, drive_output);   // HHCode
    task::sleep(10);
  }
}

/**
 * Drives to a specified point and orientation on the field.
 * Uses a boomerang controller. The carrot point is back from the target
 * by the same distance as the robot's distance to the target, times the lead. The
 * robot always tries to go to the carrot, which is constantly moving, and the
 * robot eventually gets into position. The heading correction is optimized to only
 * try to reach the correct angle when drive error is low, and the robot will drive 
 * backwards to reach a pose if it's faster. .5 is a reasonable value for the lead. 
 * The setback parameter is used to glide into position more effectively. It is
 * the distance back from the target that the robot tries to drive to first.
 * 
 * @param X_position Desired x position in inches.
 * @param Y_position Desired y position in inches.
 * @param angle Desired orientation in degrees.
 * @param lead Constant scale factor that determines how far away the carrot point is. 
 * @param setback Distance in inches from target by which the carrot is always pushed back.
 * @param drive_min_voltage Minimum voltage on the drive, used for chaining movements.
 */

void Drive::drive_to_pose(float X_position, float Y_position, float angle){
  drive_to_pose(X_position, Y_position, angle, boomerang_lead, boomerang_setback, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_pose(float X_position, float Y_position, float angle, float lead, float setback, float drive_min_voltage){
  drive_to_pose(X_position, Y_position, angle, lead, setback, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_pose(float X_position, float Y_position, float angle, float lead, float setback, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage){
  drive_to_pose(X_position, Y_position, angle, lead, setback, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}


void Drive::drive_to_pose(float X_position, float Y_position, float angle, float lead, float setback, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  drive_to_pose(X_position, Y_position, angle, lead, setback, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_pose(float X_position, float Y_position, float angle, float lead, float setback, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  // HHCode: async, chain, slew and stall support.
  if (begin_motion(MOTION_ODOM, [=]{ drive_to_pose(X_position, Y_position, angle, lead, setback, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti); })) { return; }
  if (take_chain()) { drive_settle_error = drive_chain_error; drive_settle_time = 0; }
  float previous_output = current_drive_voltage();

  float target_distance = hypot(X_position-get_X_position(),Y_position-get_Y_position());
  PID drivePID(target_distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading(), heading_kp, heading_ki, heading_kd, heading_starti);
  bool line_settled = is_line_settled(X_position, Y_position, angle, get_X_position(), get_Y_position());
  bool prev_line_settled = is_line_settled(X_position, Y_position, angle, get_X_position(), get_Y_position());
  bool crossed_center_line = false;
  bool center_line_side = is_line_settled(X_position, Y_position, angle+90, get_X_position(), get_Y_position());
  bool prev_center_line_side = center_line_side;
  while(!drivePID.is_settled()){
    line_settled = is_line_settled(X_position, Y_position, angle, get_X_position(), get_Y_position());
    if(line_settled && !prev_line_settled){ break; }
    prev_line_settled = line_settled;

    center_line_side = is_line_settled(X_position, Y_position, angle+90, get_X_position(), get_Y_position());
    if(center_line_side != prev_center_line_side){
      crossed_center_line = true;
    }

    target_distance = hypot(X_position-get_X_position(),Y_position-get_Y_position());

    float carrot_X = X_position - sin(to_rad(angle)) * (lead * target_distance + setback);
    float carrot_Y = Y_position - cos(to_rad(angle)) * (lead * target_distance + setback);

    float drive_error = hypot(carrot_X-get_X_position(),carrot_Y-get_Y_position());
    float heading_error = reduce_negative_180_to_180(to_deg(atan2(carrot_X-get_X_position(),carrot_Y-get_Y_position()))-get_absolute_heading());

    if (drive_error<drive_settle_error || crossed_center_line || drive_error < setback) { 
      heading_error = reduce_negative_180_to_180(angle-get_absolute_heading()); 
      drive_error = target_distance;
    }
    
    float drive_output = drivePID.compute(drive_error);

    float heading_scale_factor = cos(to_rad(heading_error));
    drive_output*=heading_scale_factor;
    heading_error = reduce_negative_90_to_90(heading_error);
    float heading_output = headingPID.compute(heading_error);

    drive_output = clamp(drive_output, -fabs(heading_scale_factor)*drive_max_voltage, fabs(heading_scale_factor)*drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    drive_output = slew(drive_output, previous_output, drive_slew);
    previous_output = drive_output;
    if (check_stall(drive_output)) { break; }

    drive_output = clamp_min_voltage(drive_output, drive_min_voltage);

    drive_with_voltage(left_voltage_scaling(drive_output, heading_output), right_voltage_scaling(drive_output, heading_output));
    hh::graph::record(drive_error, drive_output);   // HHCode
    task::sleep(10);
  }
}

/**
 * Turns to a specified point on the field.
 * Functions similarly to turn_to_angle() except with a point. The
 * extra_angle_deg parameter turns the robot extra relative to the 
 * desired target. For example, if you want the back of your robot
 * to point at (36, 42), you would run turn_to_point(36, 42, 180).
 * 
 * @param X_position Desired x position in inches.
 * @param Y_position Desired y position in inches.
 * @param extra_angle_deg Angle turned past the desired heading in degrees.
 */

void Drive::turn_to_point(float X_position, float Y_position){
  turn_to_point(X_position, Y_position, 0, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg){
  turn_to_point(X_position, Y_position, extra_angle_deg, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout){
  turn_to_point(X_position, Y_position, extra_angle_deg, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  // HHCode: async, chain, slew and stall support.
  if (begin_motion(MOTION_TURN, [=]{ turn_to_point(X_position, Y_position, extra_angle_deg, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti); })) { return; }
  if (take_chain()) { turn_settle_error = turn_chain_error; turn_settle_time = 0; }
  float previous_output = current_turn_voltage();

  PID turnPID(reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position())) - get_absolute_heading()), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
  while(turnPID.is_settled() == false){
    float error = reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position())) - get_absolute_heading() + extra_angle_deg);
    float output = turnPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    output = slew(output, previous_output, turn_slew);
    previous_output = output;
    if (check_stall(output)) { break; }
    drive_with_voltage(output, -output);
    hh::graph::record(error, output);   // HHCode
    task::sleep(10);
  }
}

/**
 * Drives and turns simultaneously to a desired pose.
 * Uses two PID loops, one drive and one heading to drive and turn
 * at the same time. Optimized to turn the quicker direction and only
 * exits once both PID loops have settled. It uses the heading constants
 * for heading but the turn exit conditions to settle.
 * 
 * @param X_position Desired x position in inches.
 * @param Y_position Desired y position in inches.
 * @param angle Desired ending angle in degrees.
 */

void Drive::holonomic_drive_to_pose(float X_position, float Y_position){
  holonomic_drive_to_pose(X_position, Y_position, get_absolute_heading(), drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_pose(float X_position, float Y_position, float angle){
  holonomic_drive_to_pose(X_position, Y_position, angle, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_pose(float X_position, float Y_position, float angle, float drive_max_voltage, float heading_max_voltage){
  holonomic_drive_to_pose(X_position, Y_position, angle, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_pose(float X_position, float Y_position, float angle, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  holonomic_drive_to_pose(X_position, Y_position, angle, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_pose(float X_position, float Y_position, float angle, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  // HHCode: async, chain, slew and stall support.
  if (begin_motion(MOTION_ODOM, [=]{ holonomic_drive_to_pose(X_position, Y_position, angle, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti); })) { return; }
  float holonomic_turn_settle_error = turn_settle_error;
  float holonomic_turn_settle_time = turn_settle_time;
  if (take_chain()) {
    drive_settle_error = drive_chain_error; drive_settle_time = 0;
    holonomic_turn_settle_error = turn_chain_error; holonomic_turn_settle_time = 0;
  }
  float previous_output = 0;

  PID drivePID(hypot(X_position-get_X_position(),Y_position-get_Y_position()), drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID turnPID(angle-get_absolute_heading(), heading_kp, heading_ki, heading_kd, heading_starti, holonomic_turn_settle_error, holonomic_turn_settle_time, turn_timeout);
  while( !(drivePID.is_settled() && turnPID.is_settled()) ){
    float drive_error = hypot(X_position-get_X_position(),Y_position-get_Y_position());
    float turn_error = reduce_negative_180_to_180(angle-get_absolute_heading());

    float drive_output = drivePID.compute(drive_error);
    float turn_output = turnPID.compute(turn_error);

    drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
    turn_output = clamp(turn_output, -heading_max_voltage, heading_max_voltage);

    drive_output = slew(drive_output, previous_output, drive_slew);
    previous_output = drive_output;
    if (check_stall(drive_output)) { break; }

    float heading_error = atan2(Y_position-get_Y_position(), X_position-get_X_position());

    // HHCode: sent through spin_holonomic() so mirrored autons work.
    spin_holonomic(
      drive_output*cos(to_rad(get_absolute_heading()) + heading_error - M_PI/4) + turn_output,
      drive_output*cos(-to_rad(get_absolute_heading()) - heading_error + 3*M_PI/4) - turn_output,
      drive_output*cos(-to_rad(get_absolute_heading()) - heading_error + 3*M_PI/4) + turn_output,
      drive_output*cos(to_rad(get_absolute_heading()) + heading_error - M_PI/4) - turn_output);
    hh::graph::record(drive_error, drive_output);   // HHCode
    task::sleep(10);
  }
}

/**
 * Controls a chassis with left stick throttle and right stick turning.
 * Default deadband is 5.
 */

void Drive::control_arcade(){
  float throttle = deadband(controller(primary).Axis3.value(), 5);
  float turn = deadband(controller(primary).Axis1.value(), 5);
  DriveL.spin(fwd, to_volt(throttle+turn), volt);
  DriveR.spin(fwd, to_volt(throttle-turn), volt);
}

/**
 * Controls a chassis with left stick throttle and strafe, and right stick turning.
 * Default deadband is 5.
 */

void Drive::control_holonomic(){
  float throttle = deadband(controller(primary).Axis3.value(), 5);
  float turn = deadband(controller(primary).Axis1.value(), 5);
  float strafe = deadband(controller(primary).Axis4.value(), 5);
  DriveLF.spin(fwd, to_volt(throttle+turn+strafe), volt);
  DriveRF.spin(fwd, to_volt(throttle-turn-strafe), volt);
  DriveLB.spin(fwd, to_volt(throttle+turn-strafe), volt);
  DriveRB.spin(fwd, to_volt(throttle-turn+strafe), volt);
}

/**
 * Controls a chassis with left stick left drive and right stick right drive.
 * Default deadband is 5.
 */

void Drive::control_tank(){
  float leftthrottle = deadband(controller(primary).Axis3.value(), 5);
  float rightthrottle = deadband(controller(primary).Axis2.value(), 5);
  DriveL.spin(fwd, to_volt(leftthrottle), volt);
  DriveR.spin(fwd, to_volt(rightthrottle), volt);
}

/**
 * Tracking task to run in the background.
 */

int Drive::position_track_task(){
  chassis.position_track();
  return(0);
}

/*---------------------------------------------------------------------------*/
/*  HHCode additions: async motions, chaining, slew, stall detection and     */
/*  strafing. Everything below this line is not part of JAR-Template.        */
/*---------------------------------------------------------------------------*/

/**
 * Makes the next motion return right away and run in the background, so the
 * auton can do other things while the robot drives.
 *
 *   chassis.async().drive_distance(36);
 *   chassis.wait_until(12);   // 12 inches in...
 *   intake.spin();            // ...start the intake
 *   chassis.wait();           // then wait for the drive to finish
 */

Drive& Drive::async(){
  next_async = true;
  return(*this);
}

/**
 * Makes the next motion end as soon as it gets close (drive_chain_error or
 * turn_chain_error) instead of stopping and settling, so it flows into the
 * motion after it.
 */

Drive& Drive::chain(){
  next_chain = true;
  return(*this);
}

bool Drive::take_chain(){
  bool chained = next_chain;
  next_chain = false;
  return(chained);
}

int Drive::motion_task_function(){
  chassis.in_motion_task = true;
  chassis.pending_motion();
  chassis.motion_active = false;
  return(0);
}

void Drive::record_progress_start(motion_kind kind){
  progress_kind = kind;
  progress_start_position = (get_left_position_in()+get_right_position_in())/2.0;
  progress_start_rotation = get_rotation();
  progress_start_strafe = get_strafe_position_in();
  progress_start_X = get_X_position();
  progress_start_Y = get_Y_position();
}

/**
 * Every motion calls this first. It returns true when the motion was handed
 * to a background task, in which case the caller returns immediately.
 */

bool Drive::begin_motion(motion_kind kind, std::function<void()> motion){
  if (in_motion_task){
    // This is the background task running the motion it was given.
    in_motion_task = false;
  } else {
    // A new motion replaces any background motion still running.
    stop_motion_task();
    if (next_async){
      next_async = false;
      record_progress_start(kind);
      pending_motion = motion;
      motion_active = true;
      motion_task = task(motion_task_function);
      return(true);
    }
  }
  record_progress_start(kind);
  hh::graph::begin(kind == MOTION_TURN ? "Turn" : (kind == MOTION_STRAFE ? "Strafe" : (kind == MOTION_ODOM ? "Odom drive" : "Drive")),
                   kind == MOTION_TURN ? "deg" : "in");
  stall_ms = 0;
  interfered = false;
  return(false);
}

void Drive::stop_motion_task(){
  if (motion_active){
    motion_task.stop();
    motion_active = false;
  }
  in_motion_task = false;
}

/**
 * Stops a background motion and brakes the drive. Does nothing if no
 * background motion is running.
 */

void Drive::cancel_motion(){
  bool was_moving = motion_active;
  stop_motion_task();
  if (was_moving){
    drive_stop(brake);
  }
}

bool Drive::is_moving(){
  return(motion_active);
}

void Drive::wait(){
  while (motion_active){
    task::sleep(10);
  }
}

/**
 * How far the current motion has gone: inches for drives and strafes,
 * degrees for turns and swings.
 */

float Drive::motion_progress(){
  switch (progress_kind){
    case MOTION_TURN:
      return(fabs(get_rotation() - progress_start_rotation));
    case MOTION_ODOM:
      return(hypot(get_X_position()-progress_start_X, get_Y_position()-progress_start_Y));
    case MOTION_STRAFE:
      return(fabs(get_strafe_position_in() - progress_start_strafe));
    default:
      return(fabs((get_left_position_in()+get_right_position_in())/2.0 - progress_start_position));
  }
}

void Drive::wait_until(float amount){
  while (motion_active && motion_progress() < amount){
    task::sleep(10);
  }
}

/**
 * Limits how fast the voltage may rise. Slowing down is never limited, so the
 * PID can still stop the robot on target.
 */

float Drive::slew(float target, float previous, float volts_per_second){
  if (volts_per_second <= 0) { return(target); }
  float step = volts_per_second/100.0;
  // Changing direction: drop to zero at once, then ramp up the other way.
  if (target*previous < 0) { previous = 0; }
  bool speeding_up = fabs(target) > fabs(previous);
  if (speeding_up && fabs(target-previous) > step){
    return(previous + (target > previous ? step : -step));
  }
  return(target);
}

float Drive::current_drive_voltage(){
  return((DriveL.voltage(volt)+DriveR.voltage(volt))/2.0);
}

float Drive::current_turn_voltage(){
  return((mirrored ? -1.0 : 1.0)*(DriveL.voltage(volt)-DriveR.voltage(volt))/2.0);
}

/**
 * True once the drive has been powered but not moving for stall_time.
 */

bool Drive::check_stall(float commanded_voltage){
  if (stall_time <= 0) { return(false); }
  bool stuck = fabs(commanded_voltage) > 3 && fabs(DriveL.velocity(percent)) < 3 && fabs(DriveR.velocity(percent)) < 3;
  stall_ms = stuck ? stall_ms+10 : 0;
  if (stall_ms >= stall_time){
    interfered = true;
    return(true);
  }
  return(false);
}

/**
 * Rebuilds the four holonomic wheel motors with the right cartridge. The
 * motors report degrees based on their cartridge, so this has to match for
 * strafe distances to be right.
 */

void Drive::set_holonomic_cartridge(vex::gearSetting cartridge){
  DriveLF = motor(abs(holonomic_ports[0]), cartridge, is_reversed(holonomic_ports[0]));
  DriveRF = motor(abs(holonomic_ports[1]), cartridge, is_reversed(holonomic_ports[1]));
  DriveLB = motor(abs(holonomic_ports[2]), cartridge, is_reversed(holonomic_ports[2]));
  DriveRB = motor(abs(holonomic_ports[3]), cartridge, is_reversed(holonomic_ports[3]));
}

/**
 * How far the robot has strafed, from the four wheel encoders.
 *
 * @return Sideways position in inches, positive to the right.
 */

float Drive::get_strafe_position_in(){
  float wheel_degrees = (DriveLF.position(deg) - DriveRF.position(deg) - DriveLB.position(deg) + DriveRB.position(deg))/4.0;
  return((mirrored ? -1.0 : 1.0)*wheel_degrees*drive_in_to_deg_ratio*x_drive_factor()*strafe_multiplier);
}

/**
 * Strafes a mecanum or X-drive sideways while holding a heading. Uses the
 * drive constants and exit conditions.
 *
 * @param distance Desired distance in inches, positive to the right.
 * @param heading Desired heading in degrees.
 */

void Drive::strafe_distance(float distance){
  strafe_distance(distance, get_absolute_heading(), drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::strafe_distance(float distance, float heading){
  strafe_distance(distance, heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::strafe_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage){
  strafe_distance(distance, heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::strafe_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  if (begin_motion(MOTION_STRAFE, [=]{ strafe_distance(distance, heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti); })) { return; }
  if (take_chain()) { drive_settle_error = drive_chain_error; drive_settle_time = 0; }
  float previous_output = 0;

  PID drivePID(distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(reduce_negative_180_to_180(heading - get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  float start_position = get_strafe_position_in();
  while(drivePID.is_settled() == false){
    float drive_error = distance+start_position-get_strafe_position_in();
    float heading_error = reduce_negative_180_to_180(heading - get_absolute_heading());
    float drive_output = drivePID.compute(drive_error);
    float heading_output = headingPID.compute(heading_error);

    drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    drive_output = slew(drive_output, previous_output, drive_slew);
    previous_output = drive_output;
    if (check_stall(drive_output)) { break; }

    spin_holonomic(drive_output+heading_output, -drive_output-heading_output, -drive_output+heading_output, drive_output-heading_output);
    hh::graph::record(drive_error, drive_output);   // HHCode
    task::sleep(10);
  }
}

/**
 * Follows a path of points with pure pursuit. The robot chases a point that
 * stays `lookahead` inches ahead of it along the path, which rounds off the
 * corners and keeps it moving. Needs odometry. Uses the drive and heading
 * constants, like drive_to_point().
 *
 * @param path Points to drive through, in order, in inches.
 * @param lookahead How far ahead on the path to aim, in inches.
 * @param backwards True to drive the path in reverse.
 */

void Drive::follow_path(std::vector<path_point> path){
  follow_path(path, path_lookahead, false, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::follow_path(std::vector<path_point> path, float lookahead){
  follow_path(path, lookahead, false, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::follow_path(std::vector<path_point> path, float lookahead, bool backwards){
  follow_path(path, lookahead, backwards, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::follow_path(std::vector<path_point> path, float lookahead, bool backwards, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage){
  follow_path(path, lookahead, backwards, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::follow_path(std::vector<path_point> path, float lookahead, bool backwards, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  if (path.empty()) { next_async = false; next_chain = false; return; }
  if (lookahead < 1) { lookahead = 1; }
  if (begin_motion(MOTION_ODOM, [=]{ follow_path(path, lookahead, backwards, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti); })) { return; }
  if (take_chain()) { drive_settle_error = drive_chain_error; drive_settle_time = 0; }
  float previous_output = current_drive_voltage();

  // The path starts where the robot is now.
  path.insert(path.begin(), {get_X_position(), get_Y_position()});
  if (path_smoothing > 0 && path.size() > 2) { path = smooth_path(path, path_smoothing); }
  int last = path.size()-1;
  path_point end = path[last];

  // remaining_after[i] is the path length from the end of segment i to the end of the path.
  std::vector<float> remaining_after(path.size(), 0);
  for (int i = last-2; i >= 0; i--){
    remaining_after[i] = remaining_after[i+1] + hypot(path[i+2].x-path[i+1].x, path[i+2].y-path[i+1].y);
  }

  // The finish line is square to the last segment, through the last point.
  float end_angle_deg = to_deg(atan2(end.x-path[last-1].x, end.y-path[last-1].y));
  bool prev_line_settled = is_line_settled(end.x, end.y, end_angle_deg, get_X_position(), get_Y_position());

  int segment = 0;
  float total_length = remaining_after[0] + hypot(path[1].x-path[0].x, path[1].y-path[0].y);
  PID drivePID(total_length, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(0, heading_kp, heading_ki, heading_kd, heading_starti);

  while(!drivePID.is_settled()){
    float X = get_X_position();
    float Y = get_Y_position();

    // Move on to the next segment once the end of this one is inside the
    // lookahead circle. Segments are only ever taken in order, so a path that
    // crosses itself can't make the robot skip ahead, and points closer
    // together than the lookahead distance are fine.
    while (segment < last-1 && hypot(path[segment+1].x-X, path[segment+1].y-Y) <= lookahead){
      segment++;
    }

    // Aim where the lookahead circle crosses this segment. If it doesn't (the
    // robot is off the path), aim at the end of the segment.
    path_point target = path[segment+1];
    {
      float dx = path[segment+1].x-path[segment].x;
      float dy = path[segment+1].y-path[segment].y;
      float fx = path[segment].x-X;
      float fy = path[segment].y-Y;
      float a = dx*dx + dy*dy;
      float b = 2*(fx*dx + fy*dy);
      float c = fx*fx + fy*fy - lookahead*lookahead;
      float discriminant = b*b - 4*a*c;
      if (a != 0 && discriminant >= 0){
        float t = (-b + sqrt(discriminant))/(2*a);
        if (t >= 0 && t <= 1){
          target = {path[segment].x + t*dx, path[segment].y + t*dy};
        }
      }
    }
    // Close to the end, aim at the end itself.
    bool final_approach = hypot(end.x-X, end.y-Y) <= lookahead && segment == last-1;
    if (final_approach) { target = end; }

    if (final_approach){
      bool line_settled = is_line_settled(end.x, end.y, end_angle_deg, X, Y);
      if (line_settled && !prev_line_settled) { break; }
      prev_line_settled = line_settled;
    }

    float to_target = hypot(target.x-X, target.y-Y);
    float drive_error = to_target + hypot(path[segment+1].x-target.x, path[segment+1].y-target.y) + remaining_after[segment];
    float heading_error = reduce_negative_180_to_180(to_deg(atan2(target.x-X, target.y-Y)) - get_absolute_heading() + (backwards ? 180 : 0));

    float drive_output = drivePID.compute(drive_error);
    // Slow down when not pointing at the target; never drive away from it.
    float heading_scale_factor = clamp(cos(to_rad(heading_error)), 0, 1);
    drive_output *= heading_scale_factor;
    float heading_output = headingPID.compute(heading_error);
    if (drive_error < drive_settle_error) { heading_output = 0; }

    drive_output = clamp(drive_output, -heading_scale_factor*drive_max_voltage, heading_scale_factor*drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);
    if (backwards) { drive_output = -drive_output; }

    drive_output = slew(drive_output, previous_output, drive_slew);
    previous_output = drive_output;
    if (check_stall(fabs(drive_output) + fabs(heading_output))) { break; }

    drive_output = clamp_min_voltage(drive_output, drive_min_voltage);

    drive_with_voltage(left_voltage_scaling(drive_output, heading_output), right_voltage_scaling(drive_output, heading_output));
    hh::graph::record(drive_error, drive_output);   // HHCode
    task::sleep(10);
  }
}

/*---------------------------------------------------------------------------*/
/*  HHCode: mirrored autons.                                                 */
/*---------------------------------------------------------------------------*/

/**
 * Makes the chassis behave as its own mirror image, so an auton written for
 * one side of the field runs on the other: left and right are swapped and
 * headings count the other way. hh::selector turns this on and off for autons
 * added with add_mirrored(); you don't normally call it yourself.
 */

void Drive::set_mirrored(bool mirror){
  mirrored = mirror;
  // A real forward tracking wheel is on the other side of a mirrored robot.
  float forward_distance = ForwardTracker_center_distance;
  if (mirror && drive_setup != ZERO_TRACKER_ODOM) { forward_distance = -forward_distance; }
  if (drive_setup == TANK_ONE_FORWARD_ENCODER || drive_setup == TANK_ONE_FORWARD_ROTATION || drive_setup == ZERO_TRACKER_ODOM){
    odom.set_physical_distances(forward_distance, 0);
  }
  if (drive_setup == TANK_ONE_SIDEWAYS_ENCODER || drive_setup == TANK_ONE_SIDEWAYS_ROTATION ||
  drive_setup == TANK_TWO_ENCODER || drive_setup == TANK_TWO_ROTATION ||
  drive_setup == HOLONOMIC_TWO_ENCODER || drive_setup == HOLONOMIC_TWO_ROTATION){
    odom.set_physical_distances(forward_distance, SidewaysTracker_center_distance);
  }
}

bool Drive::is_mirrored(){
  return(mirrored);
}

/**
 * The inertial sensor's rotation in degrees, not wrapped to 0-360, with the
 * gyro scale applied. Counts the other way in a mirrored auton.
 */

float Drive::get_rotation(){
  return((mirrored ? -1.0 : 1.0)*Gyro.rotation()*360.0/gyro_scale);
}

/**
 * Spins the four holonomic wheels, swapping left and right in a mirrored auton.
 */

void Drive::spin_holonomic(float lf, float rf, float lb, float rb){
  if (mirrored){
    float swap = lf; lf = rf; rf = swap;
    swap = lb; lb = rb; rb = swap;
  }
  DriveLF.spin(fwd, lf, volt);
  DriveRF.spin(fwd, rf, volt);
  DriveLB.spin(fwd, lb, volt);
  DriveRB.spin(fwd, rb, volt);
}
/*---------------------------------------------------------------------------*/
/*  HHCode: turn direction, backward constants, path smoothing.              */
/*---------------------------------------------------------------------------*/

/**
 * Makes the next turn_to_angle() go clockwise (to the right) even when the
 * other way is shorter. Useful when something is in the way.
 *
 *   chassis.clockwise().turn_to_angle(270);
 */

Drive& Drive::clockwise(){
  next_turn_direction = 1;
  return(*this);
}

Drive& Drive::counterclockwise(){
  next_turn_direction = -1;
  return(*this);
}

int Drive::take_turn_direction(){
  int direction = next_turn_direction;
  next_turn_direction = 0;
  return(direction);
}

/**
 * Gives backward drives their own constants, for robots that behave
 * differently in reverse (weight at one end, different traction). Used by
 * drive_distance(distance) and drive_distance(distance, heading) when the
 * distance is negative.
 */

void Drive::set_drive_backward_constants(float max_voltage, float kp, float ki, float kd, float starti){
  backward_max_voltage = max_voltage;
  backward_kp = kp;
  backward_ki = ki;
  backward_kd = kd;
  backward_starti = starti;
  use_backward_constants = true;
}

void Drive::clear_drive_backward_constants(){
  use_backward_constants = false;
}

/**
 * Rounds the corners of a path. Points are added every few inches, then
 * pulled toward their neighbors. The first and last points don't move.
 *
 * @param path The corner points.
 * @param smoothing 0 leaves the path alone; 0.9 is very round.
 */

std::vector<path_point> Drive::smooth_path(std::vector<path_point> path, float smoothing){
  smoothing = clamp(smoothing, 0, 0.98);
  const float spacing = 3;

  std::vector<path_point> dense;
  for (int i = 0; i+1 < (int)path.size(); i++){
    float dx = path[i+1].x-path[i].x;
    float dy = path[i+1].y-path[i].y;
    int steps = (int)(hypot(dx, dy)/spacing);
    if (steps < 1) { steps = 1; }
    for (int j = 0; j < steps; j++){
      dense.push_back({path[i].x + dx*j/steps, path[i].y + dy*j/steps});
    }
  }
  dense.push_back(path[path.size()-1]);

  std::vector<path_point> smooth = dense;
  float keep = 1 - smoothing;
  for (int pass = 0; pass < 200; pass++){
    float change = 0;
    for (int i = 1; i+1 < (int)smooth.size(); i++){
      float x = smooth[i].x + keep*(dense[i].x-smooth[i].x) + smoothing*(smooth[i-1].x+smooth[i+1].x-2*smooth[i].x);
      float y = smooth[i].y + keep*(dense[i].y-smooth[i].y) + smoothing*(smooth[i-1].y+smooth[i+1].y-2*smooth[i].y);
      change += fabs(x-smooth[i].x) + fabs(y-smooth[i].y);
      smooth[i] = {x, y};
    }
    if (change < 0.001) { break; }
  }
  return(smooth);
}
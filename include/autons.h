#pragma once
#include "JAR-Template/drive.h"

class Drive;

extern Drive chassis;

void default_constants();
void odom_constants();

// Your autons. Add a line here for each one you write in src/autons/.
void example_auton();

// JAR-Template's test autons, used for tuning.
void drive_test();
void turn_test();
void swing_test();
void full_test();
void odom_test();
void tank_odom_test();
void holonomic_odom_test();
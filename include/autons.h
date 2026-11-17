#pragma once
#include "JAR-Template/drive.h"

class Drive;

extern Drive chassis;

// src/autons/constants.cpp
void default_constants();
void odom_constants();

// src/autons/auton_list.cpp
void register_autons();

// src/autons/tests.cpp
void drive_test();
void turn_test();
void swing_test();
void full_test();
void odom_test();
void tank_odom_test();
void holonomic_odom_test();

// Declare your own autons below, one line each:


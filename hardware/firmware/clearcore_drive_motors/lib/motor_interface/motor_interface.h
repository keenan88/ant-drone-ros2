#ifndef MOTOR_INTERFACE_H
#define MOTOR_INTERFACE_H

#include <stdint.h>
#include "ClearCore.h"

// Interface to clearpath motors, using clearcore SDK

void initialize_motors();

bool CommandVelocity(MotorDriver motor, double commandedRPM);

double get_wheel_abs_radpers(MotorDriver motor, bool is_fl_motor);

void stop_motors();

#endif
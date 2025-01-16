#ifndef MOTOR_INTERFACE_H
#define MOTOR_INTERFACE_H

#include <stdint.h>
#include "ClearCore.h"

// Interface to clearpath motors, using clearcore SDK

void initialize_motors();

bool CommandVelocity(MotorDriver motor, double commandedRPM);

float get_motor_speed(MotorDriver motor);

#endif
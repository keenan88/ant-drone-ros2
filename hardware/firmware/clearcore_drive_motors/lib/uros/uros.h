#ifndef MICRO_ROS_H
#define MICRO_ROS_H

#include <rcl/rcl.h>

constexpr uint8_t kNumberOfTimers = 2; // diagnostics publish timer and system loop timer
constexpr uint8_t kNumberOfSubscriptions = 1;
constexpr uint8_t kNumberOfHandles = kNumberOfTimers + kNumberOfSubscriptions;
constexpr uint16_t kUpdateSystemStatePeriodMs = 16; // 16ms gives 60Hz
constexpr uint32_t kSerialBaudRate = 115200;

void InitializeMicroRosTransport();

void ManageAgentLifecycle();

#endif
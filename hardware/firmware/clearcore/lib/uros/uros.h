#ifndef MICRO_ROS_H
#define MICRO_ROS_H

#include <rcl/rcl.h>

constexpr uint8_t kNumberOfTimers = 2;
constexpr uint8_t kNumberOfSubscriptions = 2;
constexpr uint8_t kNumberOfHandles = kNumberOfTimers + kNumberOfSubscriptions;
constexpr uint16_t kUpdateSystemStatePeriodMs = 100;
constexpr uint32_t kSerialBaudRate = 115200;

void InitializeMicroRosTransport();

void ManageAgentLifecycle();

#endif
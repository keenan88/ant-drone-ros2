#ifndef MICRO_ROS_H
#define MICRO_ROS_H

#include <diagnostic_msgs/msg/diagnostic_array.h>
#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <diagnostic_msgs/msg/key_value.h>
#include <micro_ros_platformio.h>
#include <micro_ros_utilities/string_utilities.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>

#include "config.h"
#include "ethernet/micro_ros_transport.h"
#include "helpers.hpp"
#include "macros.h"

constexpr uint32_t kSerialBaudRate = 115200;

enum class AgentStates {
  kWaitingForConnection,
  kAvailable,
  kConnected,
  kDisconnected
};
extern enum AgentStates g_agent_state;

extern StringMessage g_debug_message;

void HandleReturnCodeError(rcl_ret_t error_code);
struct timespec GetTime();
void UpdateTimeOffsetFromAgent();

void InitializeMicroRosTransport();

bool CreateEntities();
void DestroyEntities();
void ManageAgentLifecycle();

void send_debug_str(const char *format, ...);

#endif
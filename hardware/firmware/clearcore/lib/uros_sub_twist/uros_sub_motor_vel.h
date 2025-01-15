#ifndef MICRO_ROS_CMD_VEL_H
#define MICRO_ROS_CMD_VEL_H

#include <std_msgs/msg/float32.h>
#include <micro_ros_platformio.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

const uint32_t kControlCommandTimeoutMs = 2000;

void InitializeCmdVel(rcl_node_t* ros_node,
                               rclc_executor_t* ros_executor);
void DeinitializeCmdVel(rcl_node_t* ros_node);
double get_commanded_rad_per_s();
int64_t get_previous_cmd_vel_time_ns();

#endif
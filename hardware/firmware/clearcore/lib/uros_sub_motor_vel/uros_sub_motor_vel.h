#ifndef MICRO_ROS_CMD_VEL_H
#define MICRO_ROS_CMD_VEL_H

#include <micro_ros_platformio.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <std_msgs/msg/float32.h>

const uint32_t kControlCommandTimeoutMs = 2000;

void InitializeMotorVelSub(rcl_node_t* ros_node, rclc_executor_t* ros_executor);
void DeinitializeMotorVelSub(rcl_node_t* ros_node);

double get_v1();

double get_v2();

double get_v3();

double get_v4();

#endif
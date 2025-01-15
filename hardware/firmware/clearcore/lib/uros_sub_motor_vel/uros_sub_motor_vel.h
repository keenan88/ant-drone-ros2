#ifndef MICRO_ROS_CMD_VEL_H
#define MICRO_ROS_CMD_VEL_H

#include <std_msgs/msg/float32.h>
#include <micro_ros_platformio.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/joint_state.h>

const uint32_t kControlCommandTimeoutMs = 2000;

void InitializeMotorVelSub(rcl_node_t* ros_node, rclc_executor_t* ros_executor);
void DeinitializeCmdVel(rcl_node_t* ros_node);
sensor_msgs__msg__JointState get_motor_setpoints();

#endif
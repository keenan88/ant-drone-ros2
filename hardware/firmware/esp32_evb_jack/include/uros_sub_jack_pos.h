#ifndef MICRO_ROS_JACK_SUB
#define MICRO_ROS_JACK_SUB

#include <micro_ros_platformio.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>

void InitJackPosSub(rcl_node_t* ros_node, rclc_executor_t* ros_executor);
void DeInitJackPosSub(rcl_node_t* ros_node);
int32_t get_jack_pos_setpoint();

#endif
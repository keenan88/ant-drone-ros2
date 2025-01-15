#ifndef MICRO_ROS_WHEEL_STATE_H
#define MICRO_ROS_WHEEL_STATE_H

#include <rcl/rcl.h>
#include <sensor_msgs/msg/joint_state.h>


void InitializeWheelState(rcl_node_t* ros_node);
void DeinitializeWheelState(rcl_node_t* ros_node);
void PublishWheelState(sensor_msgs__msg__JointState motor_vels);

#endif
#ifndef MICRO_ROS_WHEEL_STATE_H
#define MICRO_ROS_WHEEL_STATE_H

#include <rcl/rcl.h>
#include <sensor_msgs/msg/joint_state.h>


void InitWheelVelPub(rcl_node_t* ros_node);
void DeInitWheelVelPub(rcl_node_t* ros_node);
void PublishWheelState();

#endif
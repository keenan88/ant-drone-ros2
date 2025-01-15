#ifndef MICRO_ROS_WHEEL_STATE_H
#define MICRO_ROS_WHEEL_STATE_H

#include <rcl/rcl.h>

void InitializeWheelState(rcl_node_t* ros_node);
void DeinitializeWheelState(rcl_node_t* ros_node);
void PublishWheelState(double position, double velocity, double curr_throttle,
                       int64_t current_time_ns);

#endif
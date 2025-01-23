#ifndef MICRO_ROS_JACK_PUB
#define MICRO_ROS_JACK_PUB

#include <rcl/rcl.h>
#include <std_msgs/msg/int32.h>


void InitJackPosPub(rcl_node_t* ros_node);
void DeInitJackPosPub(rcl_node_t* ros_node);
void PublishJackPos();

#endif
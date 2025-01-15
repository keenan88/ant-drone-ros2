#include "uros_pub_wheel_state.h"

#include <micro_ros_platformio.h>
#include <micro_ros_utilities/string_utilities.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/joint_state.h>

#include "constants.h"
#include "macros.h"

rcl_publisher_t wheels_state_publisher;
sensor_msgs__msg__JointState wheels_state_msg;

void PublishWheelState(double rad_pos, double rad_vel, double curr_throttle,
                       int64_t curr_t_ns) {
  if (rmw_uros_epoch_synchronized()) {
    wheels_state_msg.position.data[0] = rad_pos;
    wheels_state_msg.velocity.data[0] = rad_vel;
    wheels_state_msg.effort.data[0] = curr_throttle;

    wheels_state_msg.header.frame_id = micro_ros_string_utilities_set(
        wheels_state_msg.header.frame_id, "base_link");

    wheels_state_msg.header.stamp.sec = (int32_t)(curr_t_ns / k_ns_per_s);
    wheels_state_msg.header.stamp.nanosec = (uint32_t)(curr_t_ns % k_ns_per_s);
  }
  RC_CHECK(rcl_publish(&wheels_state_publisher, &wheels_state_msg, NULL));
}

void InitializeWheelState(rcl_node_t *ros_node) {
  sensor_msgs__msg__JointState__init(&wheels_state_msg);

  wheels_state_msg.position.size = 1;
  wheels_state_msg.position.data =
      (double *)malloc(sizeof(double) * wheels_state_msg.position.size);

  wheels_state_msg.velocity.size = 1;
  wheels_state_msg.velocity.data =
      (double *)malloc(sizeof(double) * wheels_state_msg.velocity.size);

  wheels_state_msg.effort.size = 1;
  wheels_state_msg.effort.data =
      (double *)malloc(sizeof(double) * wheels_state_msg.effort.size);

  RC_CHECK(rclc_publisher_init_default(
      &wheels_state_publisher, ros_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      "state"));
}

void DeinitializeWheelState(rcl_node_t *ros_node) {
  sensor_msgs__msg__JointState__fini(&wheels_state_msg);
  RC_CHECK(rcl_publisher_fini(&wheels_state_publisher, ros_node));
}
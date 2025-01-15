#include "uros_pub_wheel_state.h"

#include <micro_ros_platformio.h>
#include <micro_ros_utilities/string_utilities.h>
#include <rclc/rclc.h>

#include "constants.h"
#include "macros.h"

rcl_publisher_t wheels_state_publisher;
sensor_msgs__msg__JointState wheels_state_msg;

void PublishWheelState(sensor_msgs__msg__JointState motor_vels) {

  if (rmw_uros_epoch_synchronized()) {
    wheels_state_msg = motor_vels;
    RC_CHECK(rcl_publish(&wheels_state_publisher, &wheels_state_msg, NULL));
  }

}

void InitializeWheelState(rcl_node_t *ros_node) {
  sensor_msgs__msg__JointState__init(&wheels_state_msg);

  wheels_state_msg.position.size = 4;
  wheels_state_msg.position.data = (double *)malloc(sizeof(double) * wheels_state_msg.position.size);

  wheels_state_msg.velocity.size = 4;
  wheels_state_msg.velocity.data = (double *)malloc(sizeof(double) * wheels_state_msg.velocity.size);

  RC_CHECK(rclc_publisher_init_default(
      &wheels_state_publisher, ros_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      "motor_states"));
}

void DeinitializeWheelState(rcl_node_t *ros_node) {
  sensor_msgs__msg__JointState__fini(&wheels_state_msg);
  RC_CHECK(rcl_publisher_fini(&wheels_state_publisher, ros_node));
}
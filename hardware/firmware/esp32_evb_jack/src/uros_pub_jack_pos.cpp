#include "uros_pub_jack_pos.h"
#include "jack.hpp"

#include <micro_ros_platformio.h>
#include <micro_ros_utilities/string_utilities.h>
#include <rclc/rclc.h>
#include "macros.h"

rcl_publisher_t jack_pos_publisher;

std_msgs__msg__Int32 jack_pos_msg;


void PublishJackPos() {
    jack_pos_msg.data = get_jack_pos();

    RC_CHECK(rcl_publish(&jack_pos_publisher, &jack_pos_msg, NULL));    
}

void InitJackPosPub(rcl_node_t *ros_node) {
  std_msgs__msg__Int32__init(&jack_pos_msg);

  RC_CHECK(rclc_publisher_init_default(
      &jack_pos_publisher, ros_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "jack_pos"));
}

void DeInitJackPosPub(rcl_node_t *ros_node) {
  std_msgs__msg__Int32__fini(&jack_pos_msg);
  RC_CHECK(rcl_publisher_fini(&jack_pos_publisher, ros_node));
}
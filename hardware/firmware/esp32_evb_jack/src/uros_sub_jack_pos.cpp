#include "uros_sub_jack_pos.h"
#include "macros.h"

rcl_subscription_t jack_pos_subscriber;
std_msgs__msg__Int32 jack_pos_setpoint_msg;

int64_t previous_cmd_vel_time_ms = 0;
int32_t jack_timeout_ms = 1000;

void JackPosCallback(const void* msgin) {
  previous_cmd_vel_time_ms = rmw_uros_epoch_nanos();

  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  jack_pos_setpoint_msg.data = msg->data;
}

void InitJackPosSub(rcl_node_t* ros_node, rclc_executor_t* ros_executor) {
  RC_CHECK(
    rclc_subscription_init_default(
      &jack_pos_subscriber, 
      ros_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), 
      "jack_pos_setpoint"
    )
  );

  RC_CHECK(
    rclc_executor_add_subscription(
      ros_executor, 
      &jack_pos_subscriber,
      &jack_pos_setpoint_msg,
      &JackPosCallback,
      ON_NEW_DATA)
  );
}

void DeInitJackPosSub(rcl_node_t* ros_node) {
  RC_CHECK(rcl_subscription_fini(&jack_pos_subscriber, ros_node));
}

int32_t get_jack_pos_setpoint()
{
  int64_t curr_time = rmw_uros_epoch_millis();
  bool within_timeout = curr_time && (curr_time <= previous_cmd_vel_time_ms + jack_timeout_ms);

  if(within_timeout)
  {
    return jack_pos_setpoint_msg.data;
  }
  else
  {
    return -1; // Return impossible setpoint as flag that motor command has timed out
  }
}
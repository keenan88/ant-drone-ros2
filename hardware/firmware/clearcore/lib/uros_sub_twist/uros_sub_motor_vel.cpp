#include "uros_sub_motor_vel.h"

#include "macros.h"

rcl_subscription_t cmd_vel_subscriber;

std_msgs__msg__Float32 drive_rad_per_s_msg;
int64_t previous_cmd_vel_time_ms = 0;

void CmdVelSubscriberCallback(const void* msgin) {
  previous_cmd_vel_time_ms = rmw_uros_epoch_nanos();
}

int64_t get_previous_cmd_vel_time_ns() { return previous_cmd_vel_time_ms; }

void InitializeCmdVel(rcl_node_t* ros_node, rclc_executor_t* ros_executor) {

  RC_CHECK(rclc_subscription_init_default(
      &cmd_vel_subscriber, ros_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "setpoint_rad_per_s"));

  RC_CHECK(rclc_executor_add_subscription(ros_executor, &cmd_vel_subscriber,
                                          &drive_rad_per_s_msg, &CmdVelSubscriberCallback,
                                          ON_NEW_DATA));
}

void DeinitializeCmdVel(rcl_node_t* ros_node) {
  RC_CHECK(rcl_subscription_fini(&cmd_vel_subscriber, ros_node));
}

double get_commanded_rad_per_s() { return drive_rad_per_s_msg.data; }
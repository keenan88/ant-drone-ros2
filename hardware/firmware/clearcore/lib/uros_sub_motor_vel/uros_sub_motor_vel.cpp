#include "uros_sub_motor_vel.h"



#include "macros.h"

rcl_subscription_t motor_vel_subscriber;

sensor_msgs__msg__JointState motor_vels_msg;
int64_t previous_cmd_vel_time_ms = 0;

void CmdVelSubscriberCallback(const void* msgin) {
  previous_cmd_vel_time_ms = rmw_uros_epoch_nanos();
}

void InitializeMotorVelSub(rcl_node_t* ros_node, rclc_executor_t* ros_executor) {

  sensor_msgs__msg__JointState__init(&motor_vels_msg);

  motor_vels_msg.position.size = 4;
  motor_vels_msg.position.data = (double *)malloc(sizeof(double) * motor_vels_msg.position.size);

  motor_vels_msg.velocity.size = 4;
  motor_vels_msg.velocity.data = (double *)malloc(sizeof(double) * motor_vels_msg.velocity.size);

  RC_CHECK(rclc_subscription_init_default(
      &motor_vel_subscriber, ros_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "motor_vel_setpoint"));

  RC_CHECK(rclc_executor_add_subscription(ros_executor, &motor_vel_subscriber,
                                          &motor_vels_msg, &CmdVelSubscriberCallback,
                                          ON_NEW_DATA));
}

void DeinitializeCmdVel(rcl_node_t* ros_node) {
  RC_CHECK(rcl_subscription_fini(&motor_vel_subscriber, ros_node));
}

sensor_msgs__msg__JointState get_motor_setpoints()
{
  return motor_vels_msg;
}

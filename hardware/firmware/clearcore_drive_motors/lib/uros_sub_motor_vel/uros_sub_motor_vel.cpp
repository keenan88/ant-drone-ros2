#include "uros_sub_motor_vel.h"
#include "motor_interface.h"
#include "macros.h"
#include "constants.h"

constexpr uint16_t max_speed_rpm = 510;
int64_t motor_timeout_ms = 500;

rcl_subscription_t motor_vel_subscriber;

sensor_msgs__msg__JointState motor_vels_msg;
int64_t previous_cmd_vel_time_ms = 0;

double cmd_wheel_radpers_fl;
double cmd_wheel_radpers_fr;
double cmd_wheel_radpers_rl;
double cmd_wheel_radpers_rr;

void MotorVelCallback(const void* msgin) {
  previous_cmd_vel_time_ms = rmw_uros_epoch_millis();
}

void InitWheelVelSub(rcl_node_t* ros_node, rclc_executor_t* ros_executor) {

  motor_vel_subscriber = rcl_get_zero_initialized_subscription();

  sensor_msgs__msg__JointState__init(&motor_vels_msg);

  static double asdf[4] = {0, 0, 0, 0};
  motor_vels_msg.velocity.capacity = 4;
  motor_vels_msg.velocity.data = asdf;
  motor_vels_msg.velocity.size = 0;


  RC_CHECK(
    rclc_subscription_init_best_effort(
      &motor_vel_subscriber, 
      ros_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), 
      "wheel_joint_cmds"
    )
  );

  RC_CHECK(
    rclc_executor_add_subscription(
      ros_executor, 
      &motor_vel_subscriber,
      &motor_vels_msg,
      &MotorVelCallback,
      ON_NEW_DATA)
  );
}

void DeInitWheelVelSub(rcl_node_t* ros_node) {
  RC_CHECK(rcl_subscription_fini(&motor_vel_subscriber, ros_node));
}

double get_cmd_wheel_radpers_fl()
{
  // return cmd_wheel_radpers_fl;
  return motor_vels_msg.velocity.data[0];
}

double get_cmd_wheel_radpers_fr()
{
  // return cmd_wheel_radpers_fr;
  return motor_vels_msg.velocity.data[1];
}

double get_cmd_wheel_radpers_rl()
{
  // return cmd_wheel_radpers_rl;
  return motor_vels_msg.velocity.data[2];
}

double get_cmd_wheel_radpers_rr()
{
  // return cmd_wheel_radpers_rr;
  return motor_vels_msg.velocity.data[3];
}

bool prev_wheel_cmd_within_timeout()
{
  // Deadman switch works if clearcore unplugged from computer, but not if docker containers crash
  int64_t curr_time = rmw_uros_epoch_millis();
  return curr_time && (curr_time <= previous_cmd_vel_time_ms + motor_timeout_ms);
}
#include "uros_sub_motor_vel.h"
#include "motor_interface.h"
#include "macros.h"
#include "constants.h"

rcl_subscription_t motor_vel_subscriber;

sensor_msgs__msg__JointState motor_vels_msg;
int64_t previous_cmd_vel_time_ms = 0;

#define FL_MOTOR ConnectorM0
#define FR_MOTOR ConnectorM2
#define RL_MOTOR ConnectorM3
#define RR_MOTOR ConnectorM1

double v1;
double v2;
double v3;
double v4;

void MotorVelCallback(const void* msgin) {
  // previous_cmd_vel_time_ms = rmw_uros_epoch_nanos();

  const sensor_msgs__msg__JointState *msg = (const sensor_msgs__msg__JointState *)msgin;
  
  CommandVelocity(FL_MOTOR, msg->velocity.data[0]);
  CommandVelocity(FR_MOTOR, msg->velocity.data[1]);
  CommandVelocity(RL_MOTOR, msg->velocity.data[2]);
  CommandVelocity(RR_MOTOR, msg->velocity.data[3]);

}

void InitializeMotorVelSub(rcl_node_t* ros_node, rclc_executor_t* ros_executor) {

  MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_A_DIRECT_B_PWM);

  ConnectorM0.EnableRequest(true);
  ConnectorM1.EnableRequest(true);
  ConnectorM2.EnableRequest(true);
  ConnectorM3.EnableRequest(true);

  motor_vel_subscriber = rcl_get_zero_initialized_subscription();

  sensor_msgs__msg__JointState__init(&motor_vels_msg);

  static double asdf[4] = {0, 0, 0, 0};
  motor_vels_msg.velocity.capacity = 4;
  motor_vels_msg.velocity.data = asdf;
  motor_vels_msg.velocity.size = 0;

  RC_CHECK(
    rclc_subscription_init_default(
      &motor_vel_subscriber, 
      ros_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), 
      "motor_vel_setpoint"
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

void DeinitializeMotorVelSub(rcl_node_t* ros_node) {
  RC_CHECK(rcl_subscription_fini(&motor_vel_subscriber, ros_node));
}

double get_v1()
{
  return v1;
}

double get_v2()
{
  return v2;
}

double get_v3()
{
  return v3;
}

double get_v4()
{
  return v4;
}

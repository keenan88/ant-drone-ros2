#include "uros_sub_motor_vel.h"
#include "ClearCore.h"
#include "macros.h"
#include "constants.h"

rcl_subscription_t motor_vel_subscriber;

sensor_msgs__msg__JointState motor_vels_msg;
int64_t previous_cmd_vel_time_ms = 0;

#define INPUT_A_FILTER 20


bool CommandVelocity(MotorDriver motor, int32_t commandedVelocity) {
    if (commandedVelocity > abs(max_speed_rpm)) {
        commandedVelocity = abs(max_speed_rpm);
    }
    else if(commandedVelocity < -abs(max_speed_rpm)){
      commandedVelocity = -abs(max_speed_rpm);
    } 

    if (motor.StatusReg().bit.AlertsPresent) {
        return false;
    }
 
    if (commandedVelocity >= 0) {
        motor.MotorInAState(false);
    }
    else {
        motor.MotorInAState(true);
    }

    Delay_ms(20 + INPUT_A_FILTER); // Delay while motor direction is set
 
    uint8_t dutyRequest = 255 * abs(commandedVelocity) / max_speed_rpm;
 
    motor.MotorInBDuty(dutyRequest);
 
    return true;
}

void CmdVelSubscriberCallback(const void* msgin) {
  previous_cmd_vel_time_ms = rmw_uros_epoch_nanos();

  int32_t commandedVelocity = static_cast<int32_t>(round(1.0 / 10 * max_speed_rpm));
 
  // Move at the commanded velocity.
  CommandVelocity(ConnectorM0, commandedVelocity);
  CommandVelocity(ConnectorM1, commandedVelocity);
  CommandVelocity(ConnectorM2, commandedVelocity);
  CommandVelocity(ConnectorM3, commandedVelocity);

}

void InitializeMotorVelSub(rcl_node_t* ros_node, rclc_executor_t* ros_executor) {

  MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_A_DIRECT_B_PWM);

  ConnectorM0.EnableRequest(true);
  ConnectorM1.EnableRequest(true);
  ConnectorM2.EnableRequest(true);
  ConnectorM3.EnableRequest(true);

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

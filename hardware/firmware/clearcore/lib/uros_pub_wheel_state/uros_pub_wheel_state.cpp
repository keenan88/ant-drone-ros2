#include "uros_pub_wheel_state.h"
#include "uros_sub_motor_vel.h"

#include <micro_ros_platformio.h>
#include <micro_ros_utilities/string_utilities.h>
#include <rclc/rclc.h>

#include "ClearCore.h"
#include "constants.h"
#include "macros.h"




rcl_publisher_t wheels_state_publisher;
sensor_msgs__msg__JointState wheels_state_msg;

float get_wheel_speed_pct(MotorDriver motor){
  MotorDriver::HlfbStates hlfbState = motor.HlfbState();
  float hlfbPercent = -1.0;

  if (hlfbState == MotorDriver::HLFB_HAS_MEASUREMENT) {
      hlfbPercent = motor.HlfbPercent();
  }

  return hlfbPercent;
}

void PublishWheelState(sensor_msgs__msg__JointState commanded_velocities) {

  if (rmw_uros_epoch_synchronized()) {

    float fl_wheel_speed_pct = get_wheel_speed_pct(ConnectorM0);
    float fr_wheel_speed_pct = get_wheel_speed_pct(ConnectorM1);
    float rl_wheel_speed_pct = get_wheel_speed_pct(ConnectorM2);
    float rr_wheel_speed_pct = get_wheel_speed_pct(ConnectorM3);

    bool all_speeds_read = fl_wheel_speed_pct >= 0 && 
                           fr_wheel_speed_pct >= 0 && 
                           fr_wheel_speed_pct >= 0 && 
                           rr_wheel_speed_pct >= 0;

    if(all_speeds_read)
    {
      // Use commanded velocities to determine direction of rotation. Possibly innacurate around 0 velocity, but saves us from needing external wheel encoders.
      sensor_msgs__msg__JointState commanded_motor_vels = get_motor_setpoints();

      float fl_wheel_speed = fl_wheel_speed_pct * max_speed_rpm * (commanded_motor_vels.velocity.data[0] > 0 ? 1 : -1);
      float fr_wheel_speed = fr_wheel_speed_pct * max_speed_rpm * (commanded_motor_vels.velocity.data[1] > 0 ? 1 : -1);
      float rl_wheel_speed = rl_wheel_speed_pct * max_speed_rpm * (commanded_motor_vels.velocity.data[2] > 0 ? 1 : -1);
      float rr_wheel_speed = rr_wheel_speed_pct * max_speed_rpm * (commanded_motor_vels.velocity.data[3] > 0 ? 1 : -1);

      wheels_state_msg.velocity.data[0] = fl_wheel_speed;
      wheels_state_msg.velocity.data[1] = fr_wheel_speed;
      wheels_state_msg.velocity.data[2] = rl_wheel_speed;
      wheels_state_msg.velocity.data[3] = rr_wheel_speed;

      RC_CHECK(rcl_publish(&wheels_state_publisher, &wheels_state_msg, NULL));
    }
  }
}

void InitializeWheelState(rcl_node_t *ros_node) {
  sensor_msgs__msg__JointState__init(&wheels_state_msg);

  ConnectorM0.HlfbMode(MotorDriver::HLFB_MODE_HAS_PWM);
  ConnectorM0.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);

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
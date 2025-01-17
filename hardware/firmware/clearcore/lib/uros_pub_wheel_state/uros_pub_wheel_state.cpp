#include "uros_pub_wheel_state.h"
#include "uros_sub_motor_vel.h"

#include <micro_ros_platformio.h>
#include <micro_ros_utilities/string_utilities.h>
#include <rclc/rclc.h>

#include "ClearCore.h"
#include "constants.h"
#include "macros.h"
#include <std_msgs/msg/float64_multi_array.h>
#include "motor_interface.h"


double wheel_speeds[4] = {0, 0, 0, 0};

rcl_publisher_t wheels_state_publisher;
sensor_msgs__msg__JointState wheels_state_msg;


void PublishWheelState() {

  // if (rmw_uros_epoch_synchronized()) {

    double fl_wheel_abs_radpers = get_wheel_abs_radpers(FL_MOTOR, true);
    double fr_wheel_abs_radpers = get_wheel_abs_radpers(FR_MOTOR, false);
    double rl_wheel_abs_radpers = get_wheel_abs_radpers(RL_MOTOR, false);
    double rr_wheel_abs_radpers = get_wheel_abs_radpers(RR_MOTOR, false);

    bool all_speeds_read = fl_wheel_abs_radpers >= 0 && 
                           fr_wheel_abs_radpers >= 0 && 
                           rl_wheel_abs_radpers >= 0 && 
                           rr_wheel_abs_radpers >= 0;

    // if(all_speeds_read )
    // {
      // Use commanded velocities to determine direction of rotation. Possibly innacurate around 0 velocity, but saves us from needing external wheel encoders.

      // float fl_wheel_speed = fl_wheel_speed_pct * max_speed_rpm * (commanded_motor_vels.data.data[0] > 0 ? 1 : -1);
      // float fr_wheel_speed = fr_wheel_speed_pct * max_speed_rpm * (commanded_motor_vels.data.data[1] > 0 ? 1 : -1);
      // float rl_wheel_speed = rl_wheel_speed_pct * max_speed_rpm * (commanded_motor_vels.data.data[2] > 0 ? 1 : -1);
      // float rr_wheel_speed = rr_wheel_speed_pct * max_speed_rpm * (commanded_motor_vels.data.data[3] > 0 ? 1 : -1);

      // wheels_state_msg.velocity.data[0] = get_cmd_wheel_radpers_fl();
      // wheels_state_msg.velocity.data[1] = get_cmd_wheel_radpers_fr();
      // wheels_state_msg.velocity.data[2] = get_cmd_wheel_radpers_rl();
      // wheels_state_msg.velocity.data[3] = get_cmd_wheel_radpers_rr();

      int64_t t_ns = rmw_uros_epoch_nanos();

      wheels_state_msg.header.stamp.sec = t_ns / (1000 * 1000 * 1000);
      wheels_state_msg.header.stamp.nanosec = t_ns % (1000 * 1000 * 1000);
      
      wheels_state_msg.velocity.data[0] = fl_wheel_abs_radpers;
      wheels_state_msg.velocity.data[1] = fr_wheel_abs_radpers;
      wheels_state_msg.velocity.data[2] = rl_wheel_abs_radpers;
      wheels_state_msg.velocity.data[3] = rr_wheel_abs_radpers;

      RC_CHECK(rcl_publish(&wheels_state_publisher, &wheels_state_msg, NULL));
    // }
  // }
}

void InitWheelVelPub(rcl_node_t *ros_node) {
  sensor_msgs__msg__JointState__init(&wheels_state_msg);

  wheels_state_msg.velocity.size = 4;
  wheels_state_msg.velocity.data = wheel_speeds;

  RC_CHECK(rclc_publisher_init_default(
      &wheels_state_publisher, ros_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      "wheel_joint_states"));
}

void DeInitWheelVelPub(rcl_node_t *ros_node) {
  sensor_msgs__msg__JointState__fini(&wheels_state_msg);
  RC_CHECK(rcl_publisher_fini(&wheels_state_publisher, ros_node));
}
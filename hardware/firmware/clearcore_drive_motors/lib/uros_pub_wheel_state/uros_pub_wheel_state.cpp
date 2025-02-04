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

rcl_publisher_t wheels_state_publisher;
sensor_msgs__msg__JointState wheels_state_msg;

double prev_fl_cmd_wheel_dir;
double prev_fr_cmd_wheel_dir;
double prev_rl_cmd_wheel_dir;
double prev_rr_cmd_wheel_dir;

double determine_wheel_dir(double prev_dir, double commanded_velocity)
{
  // If 0 velocity command sent, assume wheel keeps going same direction until opposite direction is sent.
  double wheel_dir = prev_dir;

  if(commanded_velocity > 0){ 
    wheel_dir = 1;
  }
  else if(commanded_velocity < 0){
    wheel_dir = -1;
  }

  return wheel_dir;
}

void PublishWheelState() {
    double fl_wheel_abs_radpers = get_wheel_abs_radpers(FL_MOTOR, true);
    double fr_wheel_abs_radpers = get_wheel_abs_radpers(FR_MOTOR, false);
    double rl_wheel_abs_radpers = get_wheel_abs_radpers(RL_MOTOR, false);
    double rr_wheel_abs_radpers = get_wheel_abs_radpers(RR_MOTOR, false);

    // Since wheel can only read abs velocity, we can use a -1 flag to indicate failed read
    bool all_speeds_read = fl_wheel_abs_radpers >= 0 && 
                           fr_wheel_abs_radpers >= 0 && 
                           rl_wheel_abs_radpers >= 0 && 
                           rr_wheel_abs_radpers >= 0;

    if(all_speeds_read)
    {
      double fl_cmd_wheel_dir = determine_wheel_dir(prev_fl_cmd_wheel_dir, get_cmd_wheel_radpers_fl());
      double fr_cmd_wheel_dir = determine_wheel_dir(prev_fr_cmd_wheel_dir, get_cmd_wheel_radpers_fr());
      double rl_cmd_wheel_dir = determine_wheel_dir(prev_rl_cmd_wheel_dir, get_cmd_wheel_radpers_rl());
      double rr_cmd_wheel_dir = determine_wheel_dir(prev_rr_cmd_wheel_dir, get_cmd_wheel_radpers_rr());

      // Use commanded velocities to determine direction of rotation. Possibly innacurate around 0 velocity, but saves us from needing external wheel encoders.

      int64_t t_ns = rmw_uros_epoch_nanos();

      wheels_state_msg.header.stamp.sec = t_ns / (1000 * 1000 * 1000);
      wheels_state_msg.header.stamp.nanosec = t_ns % (1000 * 1000 * 1000);
      
      wheels_state_msg.velocity.data[0] = fl_cmd_wheel_dir * fl_wheel_abs_radpers;
      wheels_state_msg.velocity.data[1] = fr_cmd_wheel_dir * fr_wheel_abs_radpers;
      wheels_state_msg.velocity.data[2] = rl_cmd_wheel_dir * rl_wheel_abs_radpers;
      wheels_state_msg.velocity.data[3] = rr_cmd_wheel_dir * rr_wheel_abs_radpers;

      prev_fl_cmd_wheel_dir = fl_cmd_wheel_dir;
      prev_fr_cmd_wheel_dir = fr_cmd_wheel_dir;
      prev_rl_cmd_wheel_dir = rl_cmd_wheel_dir;
      prev_rr_cmd_wheel_dir = rr_cmd_wheel_dir;

      RC_SOFT_CHECK(rcl_publish(&wheels_state_publisher, &wheels_state_msg, NULL));
    }

    
}

void InitWheelVelPub(rcl_node_t *ros_node) {
  sensor_msgs__msg__JointState__init(&wheels_state_msg);

  wheels_state_msg.velocity.size = 4;
  wheels_state_msg.velocity.data = (double*) malloc(sizeof(double) * wheels_state_msg.velocity.size); // IMPORTANT!! Use Malloc (not a static array), so that sensor_msgs__msg__JointState__fini can free this memory!!

  RC_CHECK(rclc_publisher_init_best_effort(
      &wheels_state_publisher, ros_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      "wheel_joint_states"));
}

void DeInitWheelVelPub(rcl_node_t *ros_node) {
  sensor_msgs__msg__JointState__fini(&wheels_state_msg);
  RC_CHECK(rcl_publisher_fini(&wheels_state_publisher, ros_node));
}
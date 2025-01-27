

#include "uros.h"

#include <micro_ros_platformio.h>

#include "constants.h"
#include "macros.h"
#include "uros_config.h"
// #include "uros_pub_diagnostic.h"
#include "uros_pub_wheel_state.h"
#include "uros_sub_motor_vel.h"
#include <sensor_msgs/msg/joint_state.h>
#include "motor_interface.h"


enum class AgentStates {
  kWaitingForConnection,
  kAvailable,
  kConnected,
  kDisconnected
};


enum AgentStates agent_state;

// ros core
rclc_support_t ros_support;
rcl_node_t ros_node;
rclc_executor_t ros_executor;
static rcl_allocator_t ros_allocator;
rcl_init_options_t ros_init_options;
uint64_t agent_time_offset = 0;

rcl_timer_t system_state_update_timer;



void InitializeMicroRosTransport() {

  Serial.begin(kSerialBaudRate);
  set_microros_serial_transports(Serial);
  delay(2000);
  agent_state = AgentStates::kWaitingForConnection;
}



void UpdateSystemStateCallback(rcl_timer_t *timer, int64_t last_call_time_ns) {
  if (timer != NULL) {

    // Deadman switch works if clearcore unplugged from computer, but not if docker containers crash
    if (prev_wheel_cmd_within_timeout() && !digitalReadClearCore(E_STOP_INPUT)) {
      CommandVelocity(FL_MOTOR, get_cmd_wheel_radpers_fl());
      CommandVelocity(FR_MOTOR, get_cmd_wheel_radpers_fr());
      CommandVelocity(RL_MOTOR, get_cmd_wheel_radpers_rl());
      CommandVelocity(RR_MOTOR, get_cmd_wheel_radpers_rr());
    }
    else
    {
      stop_motors();
    }

    PublishWheelState();
  }
  else
    {
      stop_motors();
    }
}

void InitSystemTimer() {
  RC_CHECK(rclc_timer_init_default(&system_state_update_timer, &ros_support,
                                   RCL_MS_TO_NS(kUpdateSystemStatePeriodMs),
                                   UpdateSystemStateCallback));
  RC_CHECK(rclc_executor_add_timer(&ros_executor, &system_state_update_timer));
}

void DeInitSystemTimer() {
  RC_CHECK(rcl_timer_fini(&system_state_update_timer));
}

void UpdateTimeOffsetFromAgent() {
  uint64_t time_now = millis();
  RC_CHECK(rmw_uros_sync_session(10));
  uint64_t ros_time_ms = rmw_uros_epoch_millis();
  agent_time_offset = ros_time_ms - time_now;
}

bool CreateEntities() {
  ros_allocator = rcl_get_default_allocator();

  ros_init_options = rcl_get_zero_initialized_init_options();
  RC_CHECK(rcl_init_options_init(&ros_init_options, ros_allocator));
  RC_CHECK(rcl_init_options_set_domain_id(&ros_init_options, kDomainId));
  rclc_support_init_with_options(&ros_support, 0, NULL, &ros_init_options,
                                 &ros_allocator);

  RC_CHECK(rclc_node_init_default(&ros_node, kNodeName, kNamespace, &ros_support));

  ros_executor = rclc_executor_get_zero_initialized_executor();
  RC_CHECK(rclc_executor_init(&ros_executor, &ros_support.context,
                              kNumberOfHandles, &ros_allocator));

  // InitializeDiagnostics(&ros_support, &ros_node,
  //                                        &ros_executor);

  InitSystemTimer();
  InitWheelVelPub(&ros_node);
  InitWheelVelSub(&ros_node, &ros_executor);

  UpdateTimeOffsetFromAgent();
  return true;
}

void DestroyEntities() {
  rmw_context_t *rmw_context =
      rcl_context_get_rmw_context(&ros_support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  // DeinitializeDiagnostics(&ros_node);
  DeInitSystemTimer();
  DeInitWheelVelPub(&ros_node);
  DeInitWheelVelSub(&ros_node);

  rclc_executor_fini(&ros_executor);
  RC_CHECK(rcl_node_fini(&ros_node));
  rclc_support_fini(&ros_support);
}

void ManageAgentLifecycle() {
  // auto reconnect to agent
  switch (agent_state) {
    case AgentStates::kWaitingForConnection:
      stop_motors();
      EXECUTE_EVERY_N_MS(500, agent_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                 ? AgentStates::kAvailable
                                 : AgentStates::kWaitingForConnection;);
      break;

    case AgentStates::kAvailable:
      stop_motors();
      agent_state = CreateEntities() ? AgentStates::kConnected : AgentStates::kWaitingForConnection;

      if (agent_state == AgentStates::kWaitingForConnection) {
        DestroyEntities();
      };

      break;

    case AgentStates::kConnected:
      EXECUTE_EVERY_N_MS(200, agent_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                 ? AgentStates::kConnected
                                 : AgentStates::kDisconnected;);

      if (agent_state == AgentStates::kConnected) {
        rclc_executor_spin_some(&ros_executor, RCL_MS_TO_NS(100));
      }
      break;

    case AgentStates::kDisconnected:
      stop_motors();
      DestroyEntities();
      agent_state = AgentStates::kWaitingForConnection;
      break;

    default:
      stop_motors();
      break;
  }
}
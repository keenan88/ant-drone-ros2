

#include "uros.h"

#include <micro_ros_platformio.h>

#include "constants.h"
#include "macros.h"
#include "uros_config.h"
#include "uros_pub_diagnostic.h"
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
rcl_allocator_t ros_allocator;
rcl_init_options_t ros_init_options;
uint64_t agent_time_offset = 0;

rcl_timer_t system_state_update_timer;

bool CreateEntities();

void InitializeMicroRosTransport() {

  IPAddress client_ip(10, 42,0,2);
  IPAddress agent_ip(10, 42,0,1);
  uint16_t agent_port = 8888;
  byte mac[6] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; // Mac addr seems to not matter?

  set_microros_native_ethernet_transports(mac, client_ip, agent_ip, agent_port);

  ConnectorUsb.Mode(Connector::USB_CDC);
  ConnectorUsb.Speed(115200);
  uint32_t timeout = 5000;
  uint32_t startTime = Milliseconds();
  ConnectorUsb.PortOpen();
  while (!ConnectorUsb && Milliseconds() - startTime < timeout) {
      continue;
  }

  agent_state = AgentStates::kWaitingForConnection;
}



void UpdateSystemStateCallback(rcl_timer_t *timer, int64_t last_call_time_ns) {
  if (timer != NULL) {

    if (prev_wheel_cmd_within_timeout() && !digitalReadClearCore(E_STOP_INPUT)) {
      CommandVelocity(FL_MOTOR, get_cmd_wheel_radpers_fl());
      CommandVelocity(FR_MOTOR, get_cmd_wheel_radpers_fr());
      CommandVelocity(RL_MOTOR, get_cmd_wheel_radpers_rl());
      CommandVelocity(RR_MOTOR, get_cmd_wheel_radpers_rr());
    }
    else
    {
      set_motors_0_vel();
    }

    PublishWheelState();
  }
  else
    {
      set_motors_0_vel();
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

  ConnectorUsb.SendLine("Creating entities");

  ros_allocator = rcl_get_default_allocator();
  
  // TODO - figure out why after 2-3 restarts of UROS agent, rclc_support_init_with_options fails.
  // ros_init_options = rcl_get_zero_initialized_init_options();
  // RC_CHECK(rcl_init_options_init(&ros_init_options, ros_allocator));
  // ConnectorUsb.SendLine("0");
  // delay(250);
  // RC_CHECK(rcl_init_options_set_domain_id(&ros_init_options, kDomainId));
  // ConnectorUsb.SendLine("1");
  // delay(250);
  // RC_CHECK(rclc_support_init_with_options(&ros_support, 0, NULL, &ros_init_options, &ros_allocator));
  // ConnectorUsb.SendLine("1.5");
  // delay(250);


  // create init_options
  RC_CHECK(rclc_support_init(&ros_support, 0, NULL, &ros_allocator));
  RC_CHECK(rclc_node_init_default(&ros_node, kNodeName, kNamespace, &ros_support));
  ros_executor = rclc_executor_get_zero_initialized_executor();
  RC_CHECK(rclc_executor_init(&ros_executor, &ros_support.context,
                              kNumberOfHandles, &ros_allocator));
                              
  InitializeDiagnostics(&ros_support, &ros_node,
                                         &ros_executor);
  InitSystemTimer();
  InitWheelVelPub(&ros_node);
  InitWheelVelSub(&ros_node, &ros_executor);

  UpdateTimeOffsetFromAgent();

  return true;
}

void DestroyEntities() {
  ConnectorUsb.SendLine("Destroying Entities");
  
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&ros_support.context);
  
  RC_CHECK(rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0));

  DeinitializeDiagnostics(&ros_node);
  DeInitSystemTimer();
  DeInitWheelVelPub(&ros_node);
  DeInitWheelVelSub(&ros_node);
  
  RC_CHECK(rclc_executor_fini(&ros_executor));
  RC_CHECK(rcl_node_fini(&ros_node));
  RC_CHECK(rclc_support_fini(&ros_support));
}

AgentStates prev_state = AgentStates::kDisconnected;
unsigned long last_print_time = 0;
bool entities_created = false;

void ManageAgentLifecycle() {
  if(prev_state != agent_state or millis() > last_print_time + 250)
  {
    ConnectorUsb.Send("Agent agent_state: ");
    char hlfbPercentStr[10];
    snprintf(hlfbPercentStr, sizeof(hlfbPercentStr), "%d", agent_state);
    ConnectorUsb.SendLine(hlfbPercentStr);
    last_print_time = millis();
  }

  switch (agent_state) {
    case AgentStates::kWaitingForConnection:
      EXECUTE_EVERY_N_MS(500, agent_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AgentStates::kAvailable : AgentStates::kWaitingForConnection;);
      break;
    case AgentStates::kAvailable:
      agent_state = (true == CreateEntities()) ? AgentStates::kConnected : AgentStates::kWaitingForConnection;
      if (agent_state == AgentStates::kWaitingForConnection) {
        set_motors_0_vel();
        DestroyEntities();
      }
      else if(agent_state == AgentStates::kConnected)
      {
        initialize_motors();
      }

      break;

    case AgentStates::kConnected:
      EXECUTE_EVERY_N_MS(200, agent_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AgentStates::kConnected : AgentStates::kDisconnected;);
      if (agent_state == AgentStates::kConnected) {
        rclc_executor_spin_some(&ros_executor, RCL_MS_TO_NS(100));
      }
      break;
    case AgentStates::kDisconnected:
      set_motors_0_vel();
      DestroyEntities();
      agent_state = AgentStates::kWaitingForConnection;
      break;
    default:
      break;
  }

  if (agent_state == AgentStates::kConnected) {
    digitalWriteClearCore(CLEARCORE_PIN_LED, HIGH);
  } else {
    digitalWriteClearCore(CLEARCORE_PIN_LED, LOW);
  }

  prev_state = agent_state;
}
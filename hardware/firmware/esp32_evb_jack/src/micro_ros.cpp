#include "micro_ros.h"
#include <std_msgs/msg/string.h>
#include "jack.hpp"
#include "uros_pub_jack_pos.h"
#include "uros_sub_jack_pos.h"

enum AgentStates g_agent_state;

rclc_support_t g_ros_support;
rcl_node_t g_ros_node;
rclc_executor_t g_ros_executor;
static rcl_allocator_t g_ros_allocator;
rcl_init_options_t g_ros_init_options;
uint64_t g_agent_time_offset = 0;

std_msgs__msg__String debug_msg;
rcl_publisher_t debug_publisher;


struct timespec GetTime() {
  struct timespec time;
  time.tv_sec = 0;
  time.tv_nsec = 0;

  uint64_t adjusted_time_ms = millis() + g_agent_time_offset;

  time.tv_sec = adjusted_time_ms / kMillisecondsInASecond;
  time.tv_nsec =
      (adjusted_time_ms % kMillisecondsInASecond) * kNanosecondsInAMillisecond;

  return time;
}

void UpdateTimeOffsetFromAgent() {
  uint64_t time_now = millis();
  RC_CHECK(rmw_uros_sync_session(10));
  uint64_t ros_time_ms = rmw_uros_epoch_millis();
  g_agent_time_offset = ros_time_ms - time_now;
}

void InitializeMicroRosTransport() {
  
  Serial.begin(kSerialBaudRate);
  set_microros_serial_transports(Serial);

  delay(2000);
  g_agent_state = AgentStates::kWaitingForConnection;
}

#define MAX_STRING_LENGTH 256

void send_debug_str(const char *format, ...) {
    char buffer[MAX_STRING_LENGTH];

    va_list args;
    va_start(args, format);

    vsnprintf(buffer, MAX_STRING_LENGTH, format, args);

    va_end(args);

    std_msgs__msg__String msg;
    rosidl_runtime_c__String c_str;
    c_str.capacity = MAX_STRING_LENGTH;
    c_str.size = strlen(buffer);
    c_str.data = buffer;
    msg.data = c_str;

    rcl_ret_t publish_result = rcl_publish(&debug_publisher, &msg, NULL);

}

void update_state(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		send_debug_str("hey");

    tick_jack();

    PublishJackPos();
	}
}

rcl_timer_t timer;

bool CreateEntities() {

  g_ros_allocator = rcl_get_default_allocator();

  g_ros_init_options = rcl_get_zero_initialized_init_options();
  RC_CHECK(rcl_init_options_init(&g_ros_init_options, g_ros_allocator));
  RC_CHECK(rcl_init_options_set_domain_id(&g_ros_init_options, kDomainId));
  rclc_support_init_with_options(&g_ros_support, 0, NULL, &g_ros_init_options,
                                 &g_ros_allocator);

  RC_CHECK(rclc_node_init_default(&g_ros_node, kNodeName, kNamespace,
                                  &g_ros_support));

  g_ros_executor = rclc_executor_get_zero_initialized_executor();
  RC_CHECK(rclc_executor_init(&g_ros_executor, &g_ros_support.context,
                              kNumberOfHandles, &g_ros_allocator));

  RC_CHECK(rclc_publisher_init_default(
    &debug_publisher, &g_ros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/jack_debug")
  );

  RC_CHECK(
    rclc_timer_init_default(
      &timer, &g_ros_support,
      RCL_MS_TO_NS(kStateUpdateMs), 
      update_state
    )
  );

  InitJackPosPub(&g_ros_node);
  InitJackPosSub(&g_ros_node, &g_ros_executor);


  RC_CHECK(rclc_executor_add_timer(&g_ros_executor, &timer));

  return true;
}

void DestroyEntities() {
  rmw_context_t *rmw_context =
      rcl_context_get_rmw_context(&g_ros_support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  RC_CHECK(rcl_timer_fini(&timer));
  RC_CHECK(rcl_publisher_fini(&debug_publisher, &g_ros_node));
  DeInitJackPosPub(&g_ros_node);
  DeInitJackPosSub(&g_ros_node);

  rclc_executor_fini(&g_ros_executor);
  RC_CHECK(rcl_node_fini(&g_ros_node));
  rclc_support_fini(&g_ros_support);
}

AgentStates prev_state;

void ManageAgentLifecycle() {

  if(prev_state != g_agent_state)
  {
    prev_state = g_agent_state;
  }

  switch (g_agent_state) {
    
    case AgentStates::kWaitingForConnection:
      EXECUTE_EVERY_N_MS(
          500, g_agent_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                   ? AgentStates::kAvailable
                                   : AgentStates::kWaitingForConnection;);
      break;
    case AgentStates::kAvailable:
      g_agent_state = (true == CreateEntities())
                          ? AgentStates::kConnected
                          : AgentStates::kWaitingForConnection;
      if (g_agent_state == AgentStates::kWaitingForConnection) {
        DestroyEntities();
      };
      break;
    case AgentStates::kConnected:
      EXECUTE_EVERY_N_MS(
          200, g_agent_state = (RMW_RET_OK == rmw_uros_ping_agent(600, 3))
                                   ? AgentStates::kConnected
                                   : AgentStates::kDisconnected;);
      if (g_agent_state == AgentStates::kConnected) {
        rclc_executor_spin_some(&g_ros_executor, RCL_MS_TO_NS(100));
      }
      break;
    case AgentStates::kDisconnected:
      DestroyEntities();
      g_agent_state = AgentStates::kWaitingForConnection;
      break;
    default:
      break;
  }

  
}
// #include "uros_pub_diagnostic.h"

// #include <diagnostic_msgs/msg/diagnostic_array.h>
// #include <diagnostic_msgs/msg/diagnostic_status.h>
// #include <std_msgs/msg/string.h>

// #include "helpers.hpp"
// #include "macros.h"
// #include "uros_config.h"

// rcl_timer_t diagnostics_publisher_timer;
// rcl_publisher_t diagnostics_publisher;

// diagnostic_msgs__msg__DiagnosticStatus diagnostics_msg;
// diagnostic_msgs__msg__KeyValue__Sequence diagnostics_key_value_sequence;
// diagnostic_msgs__msg__KeyValue diagnostics_key_value[kDiagnosticsMaxMessages];

// std_msgs__msg__String diagnostics_status_msg_name;
// std_msgs__msg__String diagnostics_status_msg_message;
// std_msgs__msg__String diagnostics_status_msg_hardware_id;
// std_msgs__msg__String firmware_version_key;
// std_msgs__msg__String firmware_version_value;
// std_msgs__msg__String firmware_build_date_key;
// std_msgs__msg__String firmware_build_date_value;
// std_msgs__msg__String uptime_key;
// std_msgs__msg__String uptime_value;
// std_msgs__msg__String debug_key;
// std_msgs__msg__String debug_value;
// StringMessage debug_message;

// void InitializeStringMsg(std_msgs__msg__String &msg) {
//   msg.data.data = (char *)malloc(kStringLength);
//   msg.data.size = 0;
//   msg.data.capacity = kStringLength;
//   msg.data.data[0] = '\0';
// }

// void UpdateStringMsgValue(std_msgs__msg__String &msg, const char *format, ...) {
//   va_list args;
//   va_start(args, format);
//   vsnprintf(msg.data.data, kStringLength, format, args);
//   va_end(args);

//   msg.data.size = strlen(msg.data.data);
// }

// void InitializeDiagnosticMessages() {
//   InitializeStringMsg(diagnostics_status_msg_name);
//   InitializeStringMsg(diagnostics_status_msg_message);
//   InitializeStringMsg(diagnostics_status_msg_hardware_id);
//   InitializeStringMsg(firmware_version_key);
//   InitializeStringMsg(firmware_version_value);
//   InitializeStringMsg(firmware_build_date_key);
//   InitializeStringMsg(firmware_build_date_value);
//   InitializeStringMsg(uptime_key);
//   InitializeStringMsg(uptime_value);
//   InitializeStringMsg(debug_key);
//   InitializeStringMsg(debug_value);

//   diagnostics_key_value_sequence.data =
//       (diagnostic_msgs__msg__KeyValue *)malloc(
//           kDiagnosticsMaxMessages * sizeof(diagnostic_msgs__msg__KeyValue));
//   diagnostics_key_value_sequence.size = 0;
//   diagnostics_key_value_sequence.capacity = kDiagnosticsMaxMessages;
// }

// void UpdateDiagnosticsMessage() {
//   UpdateStringMsgValue(diagnostics_status_msg_name, "diagnostics data");
//   UpdateStringMsgValue(diagnostics_status_msg_message, "n/a");
//   UpdateStringMsgValue(diagnostics_status_msg_hardware_id, HARDWARE_ID);
//   UpdateStringMsgValue(firmware_version_key, "firmware version");
//   UpdateStringMsgValue(firmware_version_value, FIRMWARE_VERSION);
//   UpdateStringMsgValue(firmware_build_date_key, "compile date");
//   UpdateStringMsgValue(firmware_build_date_value, __DATE__);
//   UpdateStringMsgValue(uptime_key, "uptime (ms)");
//   UpdateStringMsgValue(uptime_value, "%lu", millis());
//   UpdateStringMsgValue(debug_key, "debug");
//   UpdateStringMsgValue(debug_value, "%s", debug_message.c_str());

//   diagnostics_key_value[0].key = firmware_version_key.data;
//   diagnostics_key_value[0].value = firmware_version_value.data;
//   diagnostics_key_value[1].key = firmware_build_date_key.data;
//   diagnostics_key_value[1].value = firmware_build_date_value.data;
//   diagnostics_key_value[2].key = uptime_key.data;
//   diagnostics_key_value[2].value = uptime_value.data;
//   diagnostics_key_value[3].key = debug_key.data;
//   diagnostics_key_value[3].value = debug_value.data;

//   diagnostics_key_value_sequence.data = diagnostics_key_value;
//   diagnostics_key_value_sequence.size = kDiagnosticsMaxMessages;

//   diagnostics_msg.level = diagnostic_msgs__msg__DiagnosticStatus__OK;
//   diagnostics_msg.name = diagnostics_status_msg_name.data;
//   diagnostics_msg.message = diagnostics_status_msg_message.data;
//   diagnostics_msg.hardware_id = diagnostics_status_msg_hardware_id.data;
//   diagnostics_msg.values = diagnostics_key_value_sequence;
// }

// void PublishDiagnosticsCallback(rcl_timer_t *timer, int64_t last_call_time_ns) {
//   (void)last_call_time_ns;
//   if (timer != NULL) {
//     UpdateDiagnosticsMessage();
//     RC_CHECK(rcl_publish(&diagnostics_publisher, &diagnostics_msg, NULL));
//   }
// }

// void InitializeDiagnostics(rclc_support_t *ros_support,
//                                             rcl_node_t *ros_node,
//                                             rclc_executor_t *ros_executor) {
//   InitializeDiagnosticMessages();                                            
//   RC_CHECK(rclc_publisher_init_default(
//       &diagnostics_publisher, ros_node,
//       ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticStatus),
//       "diagnostics"));

//   RC_CHECK(rclc_timer_init_default(&diagnostics_publisher_timer, ros_support,
//                                    RCL_MS_TO_NS(kDiagnosticsPublisherPeriodMs),
//                                    PublishDiagnosticsCallback));

//   RC_CHECK(rclc_executor_add_timer(ros_executor, &diagnostics_publisher_timer));
// }

// void DeinitializeDiagnostics(rcl_node_t *ros_node) {
//   RC_CHECK(rcl_timer_fini(&diagnostics_publisher_timer));
//   RC_CHECK(rcl_publisher_fini(&diagnostics_publisher, ros_node));
// }

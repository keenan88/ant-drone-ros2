#include "nodes.h"

CheckHeartbeat::CheckHeartbeat(const std::string &name, const NodeConfig &conf,
                               const RosNodeParams &params)
    : RosServiceNode<antdrone_interfaces::srv::MissionHeartbeatSrv>(name, conf,
                                                                    params) {}

PortsList CheckHeartbeat::providedPorts() {

  return {OutputPort<std::string>("error_state")};
}

bool CheckHeartbeat::setRequest(Request::SharedPtr &request) {
  // must return true if we are ready to send the request
  return true;
}

NodeStatus
CheckHeartbeat::onResponseReceived(const Response::SharedPtr &response) {
  NodeStatus node_status = NodeStatus::FAILURE;

  if (response->heartbeat_timeout_healthy) {
    node_status = NodeStatus::SUCCESS;
    if (auto node = node_.lock()) {
      RCLCPP_INFO(node->get_logger(), "[%s] Heartbeat timeout healthy ",
                  this->name().c_str());
    }
  } else {
    if (auto node = node_.lock()) {
      RCLCPP_INFO(node->get_logger(), "[%s] Heartbeat timeout not healthy ",
                  this->name().c_str());
    }
    setOutput("error_state", "heartbeat_timed_out");
  }

  return node_status;
}

NodeStatus CheckHeartbeat::onFailure(ServiceNodeErrorCode error) {
  NodeStatus node_status = NodeStatus::FAILURE;

  if (auto node = node_.lock()) {
    RCLCPP_INFO(node->get_logger(),
                "[%s] Error calling drone heartbeat server ",
                this->name().c_str());
  }
  setOutput("error_state", "heartbeat_service_call_failed");

  return node_status;
}
#include "nodes.h"

GoUnderWorker::GoUnderWorker(const std::string &name, const NodeConfig &conf, const RosNodeParams &params) : RosServiceNode<antdrone_interfaces::srv::GoUnder>(name, conf, params) {}

PortsList GoUnderWorker::providedPorts() { return {}; }

bool GoUnderWorker::setRequest(Request::SharedPtr &request) {
  
  return true;
}

NodeStatus GoUnderWorker::onResponseReceived(const Response::SharedPtr &response) {
  if (auto node = node_.lock()) {
    RCLCPP_INFO(node->get_logger(), "[%s] Go under worker response: %d, %s" 
                , this->name().c_str(), response -> success, (response->side_entered).c_str());
  }

  if (response->success) {
    return NodeStatus::SUCCESS;
  } else {
    return NodeStatus::FAILURE;
  }

  return NodeStatus::RUNNING;
}

NodeStatus GoUnderWorker::onFailure(ServiceNodeErrorCode error) {
  if (error) {
  } // To avoid build warning
  if (auto node = node_.lock()) {
    RCLCPP_INFO(node->get_logger(), "[%s] Go under worker service call failed", this->name().c_str());
  }
  return NodeStatus::FAILURE;
}
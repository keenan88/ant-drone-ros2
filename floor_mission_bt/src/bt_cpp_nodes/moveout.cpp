#include "nodes.h"

MoveOut::MoveOut(const std::string &name, const NodeConfig &conf,
                 const RosNodeParams &params)
    : RosServiceNode<ant_queen_interfaces::srv::MoveOut>(name, conf, params) {}

PortsList MoveOut::providedPorts() { return {}; }

bool MoveOut::setRequest(Request::SharedPtr &request) {
  if (auto node = node_.lock()) {
    RCLCPP_INFO(node->get_logger(), "[%s] Moving out from under worker..",
                this->name().c_str());
  }
  return true;
}

NodeStatus MoveOut::onResponseReceived(const Response::SharedPtr &response) {
  if (auto node = node_.lock()) {
    RCLCPP_INFO(node->get_logger(), "[%s] MoveOut complete ? %d",
                this->name().c_str(), response->success);
  }

  if (response->success) {
    return NodeStatus::SUCCESS;
  }

  return NodeStatus::FAILURE;
}

NodeStatus MoveOut::onFailure(ServiceNodeErrorCode error) {
  if (auto node = node_.lock()) {
    RCLCPP_INFO(node->get_logger(), "[%s] Error contacting moveout server",
                this->name().c_str());
  }
  return NodeStatus::FAILURE;
}
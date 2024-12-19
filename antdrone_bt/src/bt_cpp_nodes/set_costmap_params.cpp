#include "nodes.h"

SetCostmapParams::SetCostmapParams(const std::string &name, const NodeConfig &conf, const RosNodeParams &params) : RosServiceNode<rcl_interfaces::srv::SetParameters>(name, conf, params) {}

PortsList SetCostmapParams::providedPorts() { return {BT::InputPort<bool>("is_carrying_worker")}; }

bool SetCostmapParams::setRequest(Request::SharedPtr &request) {

  auto is_carrying_worker_result = getInput<bool>("is_carrying_worker");

  if (!is_carrying_worker_result) {
    std::cerr << "Couldn't get is_carrying_worker!" << std::endl;
  } else {
    bool is_carrying_worker = bool(is_carrying_worker_result.value());

    rcl_interfaces::msg::ParameterValue inflation_radius_param_value;

    rcl_interfaces::msg::Parameter inflation_radius_param;
    inflation_radius_param.name = "inflation_layer.inflation_radius";

    if (is_carrying_worker) {
      inflation_radius_param_value.type = 3;
      inflation_radius_param_value.double_value = 0.8;
    } else {
      inflation_radius_param_value.type = 3;
      inflation_radius_param_value.double_value = 0.5;
    }

    inflation_radius_param.value = inflation_radius_param_value;
    request->parameters = {inflation_radius_param};

    if (auto node = node_.lock()) {
      RCLCPP_INFO(node->get_logger(), "[%s] Sending costmap set inflation request ", this->name().c_str());
    }

    return true;
  }

  return false;
}

NodeStatus SetCostmapParams::onResponseReceived(const Response::SharedPtr &response) {
  if (auto node = node_.lock()) {
    RCLCPP_INFO(node->get_logger(), "[%s] Costmap set results:  %d, %s", this->name().c_str(), response->results[0].successful, response->results[0].reason.c_str());
  }

  if (response->results[0].successful) {
    return NodeStatus::SUCCESS;
  }

  return NodeStatus::FAILURE;
}

NodeStatus SetCostmapParams::onFailure(ServiceNodeErrorCode error) {
  if (error) {
  } // To avoid build warning
  return NodeStatus::FAILURE;
}
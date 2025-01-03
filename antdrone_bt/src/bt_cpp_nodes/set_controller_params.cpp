#include "nodes.h"

SetNav2ControllerParams::SetNav2ControllerParams(const std::string &name, const NodeConfig &conf, const RosNodeParams &params) : RosServiceNode<rcl_interfaces::srv::SetParameters>(name, conf, params) {}

PortsList SetNav2ControllerParams::providedPorts() { 
  return {
    BT::InputPort<float>("vy_max"),
    BT::InputPort<float>("wz_max"),
    BT::InputPort<float>("xy_goal_tolerance"),
    BT::InputPort<float>("yaw_goal_tolerance")
  }; 
}

bool SetNav2ControllerParams::setRequest(Request::SharedPtr &request) {
  
  rcl_interfaces::msg::ParameterValue vy_max_param_val;
  rcl_interfaces::msg::Parameter vy_max_param;
  vy_max_param.name = "FollowPath.vy_max";
  getInput("vy_max", vy_max_param_val.double_value);
  vy_max_param_val.type = 3;
  vy_max_param.value = vy_max_param_val;

  rcl_interfaces::msg::ParameterValue wz_max_param_val;
  rcl_interfaces::msg::Parameter wz_max_param;
  wz_max_param.name = "FollowPath.wz_max";
  getInput("wz_max", wz_max_param_val.double_value);
  wz_max_param_val.type = 3;
  wz_max_param.value = wz_max_param_val;

  rcl_interfaces::msg::ParameterValue xy_goal_tolerance_param_val;
  rcl_interfaces::msg::Parameter xy_goal_tolerance_param;
  xy_goal_tolerance_param.name = "goal_checker.xy_goal_tolerance";
  getInput("xy_goal_tolerance", xy_goal_tolerance_param_val.double_value);
  xy_goal_tolerance_param_val.type = 3;
  xy_goal_tolerance_param.value = xy_goal_tolerance_param_val;

  rcl_interfaces::msg::ParameterValue yaw_goal_tolerance_param_val;
  rcl_interfaces::msg::Parameter yaw_goal_tolerance_param;
  yaw_goal_tolerance_param.name = "goal_checker.yaw_goal_tolerance";
  getInput("yaw_goal_tolerance", yaw_goal_tolerance_param_val.double_value);
  yaw_goal_tolerance_param_val.type = 3;
  yaw_goal_tolerance_param.value = yaw_goal_tolerance_param_val;

  request->parameters = {vy_max_param, wz_max_param, xy_goal_tolerance_param, yaw_goal_tolerance_param};

  if (auto node = node_.lock()) {
    RCLCPP_INFO(node->get_logger(), "[%s] Sending nav2 controller vy_max, vw_max, yaw_goal_tolerance, and xy_goal_tolerance request", this->name().c_str());
  }

  return true;
}

NodeStatus SetNav2ControllerParams::onResponseReceived(const Response::SharedPtr &response) {
  if (auto node = node_.lock()) {
    RCLCPP_INFO(node->get_logger(), "[%s] Costmap set results:  %d, %s", this->name().c_str(), response->results[0].successful, response->results[0].reason.c_str());
  }

  if (response->results[0].successful) {
    return NodeStatus::SUCCESS;
  }

  return NodeStatus::FAILURE;
}

NodeStatus SetNav2ControllerParams::onFailure(ServiceNodeErrorCode error) {
  if (error) {
  } // To avoid build warning
  return NodeStatus::FAILURE;
}
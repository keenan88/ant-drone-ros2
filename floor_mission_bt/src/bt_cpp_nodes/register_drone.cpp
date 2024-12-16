#include "nodes.h"

RegisterDrone::RegisterDrone(const std::string &name, const NodeConfig &conf,
                             const RosNodeParams &params)
    : RosServiceNode<ant_queen_interfaces::srv::RegisterRobot>(name, conf,
                                                               params) {}

PortsList RegisterDrone::providedPorts() {

  return {InputPort<std::string>("drone_name")};
}

bool RegisterDrone::setRequest(Request::SharedPtr &request) {
  getInput("drone_name", request->robot_name);

  request->state = "IDLE";

  if (auto node = node_.lock()) {
    request->heartbeat_s = (int64_t)node->get_clock()->now().seconds();
  }

  // must return true if we are ready to send the request
  return true;
}

NodeStatus
RegisterDrone::onResponseReceived(const Response::SharedPtr &response) {
  NodeStatus node_status = NodeStatus::FAILURE;

  if (response->registered) {
    node_status = NodeStatus::SUCCESS;
    if (auto node = node_.lock()) {
      RCLCPP_INFO(node->get_logger(), "[%s] Drone registered with queen ",
                  this->name().c_str());
    }
  } else {
    if (auto node = node_.lock()) {
      RCLCPP_INFO(node->get_logger(),
                  "[%s] Drone could not be registered with queen ",
                  this->name().c_str());
    }
  }

  return node_status;
}

NodeStatus RegisterDrone::onFailure(ServiceNodeErrorCode error) {
  if(error){} // To avoid build warning
  NodeStatus node_status = NodeStatus::FAILURE;

  if (auto node = node_.lock()) {
    RCLCPP_INFO(node->get_logger(), "[%s] Error calling register_robot server ",
                this->name().c_str());
  }

  return node_status;
}
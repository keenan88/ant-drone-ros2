#include "nodes.h"

using namespace BT;

CheckGoToWaypointSuccess::CheckGoToWaypointSuccess(const std::string &name, const NodeConfig &conf, const RosNodeParams &params)
    : RosServiceNode<ant_queen_interfaces::srv::LastKnownEndWaypointName>(name, conf, params){};

PortsList CheckGoToWaypointSuccess::providedPorts() {

  return providedBasicPorts({
      BT::InputPort<std::string>("vertex_name"),
      BT::InputPort<std::string>("drone_name"),
      BT::OutputPort<std::string>("error_state"),
  });
}

bool CheckGoToWaypointSuccess::setRequest(Request::SharedPtr &request) {
  getInput("drone_name", request->drone_name);
  getInput("vertex_name", desired_vertex_name);
  // must return true if we are ready to send the request
  return true;
}

NodeStatus CheckGoToWaypointSuccess::onResponseReceived(const Response::SharedPtr &response) {
  NodeStatus node_status = NodeStatus::FAILURE;

  if (response->last_known_waypoint_name == desired_vertex_name) {
    node_status = NodeStatus::SUCCESS;
    if (auto node = node_.lock()) {
      RCLCPP_INFO(node->get_logger(), "[%s] Drone has reached desired vertex %s ", this->name().c_str(), desired_vertex_name.c_str());
    }
  }

  return node_status;
}

NodeStatus CheckGoToWaypointSuccess::onFailure(ServiceNodeErrorCode error) {
  if (error) {
  } // To avoid build warning
  NodeStatus node_status = NodeStatus::FAILURE;

  if (auto node = node_.lock()) {
    RCLCPP_INFO(node->get_logger(), "[%s] Error calling drone last_known_end_waypoint_name server ", this->name().c_str());
  }
  setOutput("error_state", "last_known_end_waypoint_name_call_failed");

  return node_status;
}

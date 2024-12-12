#include "nodes.h"

using namespace BT;


CheckGoToPlaceSuccess::CheckGoToPlaceSuccess(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)  : 
  RosServiceNode<ant_queen_interfaces::srv::LastKnownEndWaypointName>(name, conf, params) {};


PortsList CheckGoToPlaceSuccess::providedPorts(){

  return providedBasicPorts ({
    BT::InputPort<std::string>("vertex_name"),
    BT::OutputPort<std::string>("error_state")
  });

}


bool CheckGoToPlaceSuccess::setRequest(Request::SharedPtr& request) {

  getInput("vertex_name", desired_vertex_name);
  // must return true if we are ready to send the request
  return true;
}


NodeStatus CheckGoToPlaceSuccess::onResponseReceived(const Response::SharedPtr& response) {
  NodeStatus node_status = NodeStatus::FAILURE;

  if(response -> last_known_waypoint_name == desired_vertex_name)
  {
    node_status = NodeStatus::SUCCESS;
    if (auto node = node_.lock())  // Attempt to lock the weak_ptr
    {
        RCLCPP_INFO(node->get_logger(), "[%s] Drone has reached desired vertex %s ", this->name().c_str(), desired_vertex_name.c_str());
    }
  }
  else
  {
    if (auto node = node_.lock())  // Attempt to lock the weak_ptr
    {
        RCLCPP_INFO(node->get_logger(), "[%s] Last known end vertex: %s, desired: %s ", this->name().c_str(), response -> last_known_waypoint_name.c_str(), desired_vertex_name.c_str());
    }
  }

  return node_status;
}


NodeStatus CheckGoToPlaceSuccess::onFailure(ServiceNodeErrorCode error) {
  NodeStatus node_status = NodeStatus::FAILURE;

  if (auto node = node_.lock())  // Attempt to lock the weak_ptr
  {
      RCLCPP_INFO(node->get_logger(), "[%s] Error calling drone last_known_end_waypoint_name server ", this->name().c_str());
  }
  setOutput("error_state", "last_known_end_waypoint_name_call_failed");

  return node_status;
}









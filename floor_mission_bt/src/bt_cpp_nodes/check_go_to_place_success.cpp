#include "nodes.h"


CheckGoToPlaceSuccess::CheckGoToPlaceSuccess(const std::string& name, const NodeConfig& conf, const RosNodeParams& params) : 
RosServiceNode<ant_fleet_interfaces::srv::LastKnownEndWaypointName>(name, conf, params) {}

PortsList CheckGoToPlaceSuccess::providedPorts()
{

  return {
    InputPort<std::string>("vertex_name"),
    OutputPort<std::string>("error_state")
  };

}

bool CheckGoToPlaceSuccess::setRequest(Request::SharedPtr& request)
{
  auto desired_vertex_name_result = getInput<std::string>("vertex_name");
  if (!desired_vertex_name_result)
  {
      if (auto node = node_.lock())  // Attempt to lock the weak_ptr
      {
          RCLCPP_INFO(node->get_logger(), "[%s] Could not read vertex_name from blackboard ", this->name().c_str());
      }
      setOutput("error_state", "couldnt_read_desired_vertex_name");
      return false;
  }

  desired_vertex_name = desired_vertex_name_result.value();
    
  // must return true if we are ready to send the request
  return true;
}

NodeStatus CheckGoToPlaceSuccess::onResponseReceived(const Response::SharedPtr& response)
{
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

NodeStatus CheckGoToPlaceSuccess::onFailure(ServiceNodeErrorCode error)
{
  NodeStatus node_status = NodeStatus::FAILURE;

  if (auto node = node_.lock())  // Attempt to lock the weak_ptr
  {
      RCLCPP_INFO(node->get_logger(), "[%s] Error calling drone last_known_end_waypoint_name server ", this->name().c_str());
  }
  setOutput("error_state", "last_known_end_waypoint_name_call_failed");

  return node_status;
}
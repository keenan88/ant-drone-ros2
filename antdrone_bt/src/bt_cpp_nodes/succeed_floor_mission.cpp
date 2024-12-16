#include "nodes.h"

SucceedFloorMission::SucceedFloorMission(const std::string& name, const NodeConfig& conf, const RosNodeParams& params) : 
RosServiceNode<ant_queen_interfaces::srv::MissionSuccess>(name, conf, params) 
{
}

PortsList SucceedFloorMission::providedPorts()
{

  return {
    InputPort<std::string>("drone_name")
  };

}

bool SucceedFloorMission::setRequest(Request::SharedPtr& request)
{

  getInput("drone_name", request -> robot_name);
  request -> mission_type = "floor_mission";

  // must return true if we are ready to send the request
  return true;
}

NodeStatus SucceedFloorMission::onResponseReceived(const Response::SharedPtr& response)
{
  if(response)
  {
    return NodeStatus::SUCCESS;
  }

  

  return NodeStatus::FAILURE;
}

NodeStatus SucceedFloorMission::onFailure(ServiceNodeErrorCode error)
{
  if(error){} // To avoid build warning
  // RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
  return NodeStatus::FAILURE;
}
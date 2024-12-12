#include "nodes.h"

SendFloorMissionSuccess::SendFloorMissionSuccess(const std::string& name, const NodeConfig& conf, const RosNodeParams& params) : 
RosServiceNode<ant_queen_interfaces::srv::MissionSuccess>(name, conf, params) 
{
}

PortsList SendFloorMissionSuccess::providedPorts()
{

  return {
    InputPort<std::string>("drone_name")
  };

}

bool SendFloorMissionSuccess::setRequest(Request::SharedPtr& request)
{

  getInput("drone_name", request -> robot_name);
  request -> mission_type = "floor_mission";

  // must return true if we are ready to send the request
  return true;
}

NodeStatus SendFloorMissionSuccess::onResponseReceived(const Response::SharedPtr& response)
{
  if(response)
  {
    return NodeStatus::SUCCESS;
  }

  

  return NodeStatus::FAILURE;
}

NodeStatus SendFloorMissionSuccess::onFailure(ServiceNodeErrorCode error)
{
  // RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
  return NodeStatus::FAILURE;
}
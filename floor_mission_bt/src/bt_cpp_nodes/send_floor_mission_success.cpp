#include "nodes.h"


SendFloorMissionSuccess::SendFloorMissionSuccess(const std::string& name, const NodeConfig& conf, const RosNodeParams& params) : 
RosServiceNode<ant_fleet_interfaces::srv::MissionSuccess>(name, conf, params) {}

PortsList SendFloorMissionSuccess::providedPorts()
{

  return {
    OutputPort<std::string>("drone_floor_mission_status"),
    InputPort<std::string>("drone_name")
  };

}

bool SendFloorMissionSuccess::setRequest(Request::SharedPtr& request)
{

  std::string drone_name;

  getInput("drone_name", drone_name);

  request -> robot_name = drone_name;
    
  // must return true if we are ready to send the request
  return true;
}

NodeStatus SendFloorMissionSuccess::onResponseReceived(const Response::SharedPtr& response)
{
  if(response)
  {
    setOutput("drone_floor_mission_status", "IDLE");
    return NodeStatus::SUCCESS;
  }

  

  return NodeStatus::FAILURE;
}

NodeStatus SendFloorMissionSuccess::onFailure(ServiceNodeErrorCode error)
{
  // RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
  return NodeStatus::FAILURE;
}
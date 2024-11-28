#include "nodes.h"


SendFloorMissionSuccess::SendFloorMissionSuccess(const std::string& name, const NodeConfig& conf, const RosNodeParams& params) : 
RosServiceNode<ant_fleet_interfaces::srv::SendFloorMissionSuccess>(name, conf, params) {}

PortsList SendFloorMissionSuccess::providedPorts()
{

  return {
    OutputPort<std::string>("drone_mission_state");
  };

}

bool SendFloorMissionSuccess::setRequest(Request::SharedPtr& request)
{
  
  auto drone_name_result = getInput<std::string>("drone_name");
  if (!drone_name_result)
  {
      return false;
  }

  request -> robot_name = drone_name_result.value();
    
  // must return true if we are ready to send the request
  return true;
}

NodeStatus SendFloorMissionSuccess::onResponseReceived(const Response::SharedPtr& response)
{
  if(response -> is_floor_mission_triggered)
  {
    setOutput("drone_mission_state", "Idle");
    return NodeStatus::SUCCESS;
  }

  

  return NodeStatus::FAILURE;
}

NodeStatus SendFloorMissionSuccess::onFailure(ServiceNodeErrorCode error)
{
  // RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
  return NodeStatus::FAILURE;
}
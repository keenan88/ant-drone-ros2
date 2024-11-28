#include "nodes.h"


SendHeartbeatToQueen::SendHeartbeatToQueen(const std::string& name, const NodeConfig& conf, const RosNodeParams& params) : 
RosServiceNode<ant_fleet_interfaces::srv::MissionHeartbeat>(name, conf, params) {}

PortsList SendHeartbeatToQueen::providedPorts()
{

  return {
    InputPort<std::string>("drone_name"),
    InputPort<std::string>("drone_floor_mission_status"),

    OutputPort<std::string>("error_state"),
  };

}

bool SendHeartbeatToQueen::setRequest(Request::SharedPtr& request)
{
  auto drone_name_result = getInput<std::string>("drone_name");
  if (!drone_name_result)
  {
      setOutput("error_state", "drone_name_unavailable");
      return false;
  }

  auto drone_status_result = getInput<std::string>("drone_status");
  if (!drone_status_result)
  {
      setOutput("error_state", "drone_status_unavailable");
      return false;
  }

  request -> robot_name = drone_name_result.value();
  request -> drone_floor_mission_status = drone_status_result.value();
  if (auto node = node_.lock()) {
    request -> robot_heartbeat_s = node->get_clock()->now().seconds();
  }
    
  // must return true if we are ready to send the request
  return true;
}

NodeStatus SendHeartbeatToQueen::onResponseReceived(const Response::SharedPtr& response)
{
  NodeStatus node_status = NodeStatus::SUCCESS;

  if(!(response -> robot_status_queen_status_match))
  {
      setOutput("error_state", "mission_statuses_different");
      node_status = NodeStatus::FAILURE;
  }

  if(last_heartbeat_time_s == -1)
  {
     last_heartbeat_time_s = response -> queen_heartbeat_s;
  }

  int64_t s_since_last_heartbeat = response -> queen_heartbeat_s - last_heartbeat_time_s;

  if (s_since_last_heartbeat > heartbeat_timeout_s){
      setOutput("error_state", "heartbeat_timeout");
      node_status = NodeStatus::FAILURE;
  }

  last_heartbeat_time_s = response -> queen_heartbeat_s;

  return node_status;
}

NodeStatus SendHeartbeatToQueen::onFailure(ServiceNodeErrorCode error)
{
  // RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
  setOutput("error_state", "heartbeat_send_failure");
  return NodeStatus::FAILURE;
}
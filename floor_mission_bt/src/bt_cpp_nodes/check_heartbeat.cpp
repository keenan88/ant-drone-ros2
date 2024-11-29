#include "nodes.h"


CheckHeartbeat::CheckHeartbeat(const std::string& name, const NodeConfig& conf, const RosNodeParams& params) : 
RosServiceNode<ant_fleet_interfaces::srv::MissionHeartbeatSrv>(name, conf, params) {}

PortsList CheckHeartbeat::providedPorts()
{

  return {
    InputPort<std::string>("drone_floor_mission_status"),
    OutputPort<std::string>("error_state")
  };

}

bool CheckHeartbeat::setRequest(Request::SharedPtr& request)
{
  auto drone_floor_mission_status_result = getInput<std::string>("drone_floor_mission_status");
  if (!drone_floor_mission_status_result)
  {
      if (auto node = node_.lock())  // Attempt to lock the weak_ptr
      {
          RCLCPP_INFO(node->get_logger(), "[%s] Could not read drone_floor_mission_status from blackboard ", this->name().c_str());
      }
      setOutput("error_state", "couldnt_read_drone_floor_mission_status");
      return false;
  }

  request -> drone_floor_mission_status = drone_floor_mission_status_result.value();
    
  // must return true if we are ready to send the request
  return true;
}

NodeStatus CheckHeartbeat::onResponseReceived(const Response::SharedPtr& response)
{
  NodeStatus node_status = NodeStatus::FAILURE;

  if(response -> robot_floor_mission_status_healthy)
  {
    if(response -> heartbeat_timeout_healthy)
    {
      node_status = NodeStatus::SUCCESS;
    }
    else 
    {
      if (auto node = node_.lock())  // Attempt to lock the weak_ptr
      {
          RCLCPP_INFO(node->get_logger(), "[%s] Heartbeat timeout not healthy ", this->name().c_str());
      }
      setOutput("error_state", "heartbeat_timed_out");
    }
  }
  else
  {
    if (auto node = node_.lock())  // Attempt to lock the weak_ptr
    {
        RCLCPP_INFO(node->get_logger(), "[%s] Heartbeat status not healthy ", this->name().c_str());
    }
    setOutput("error_state", "floor_mission_status_mismatch");
  }

  return node_status;
}

NodeStatus CheckHeartbeat::onFailure(ServiceNodeErrorCode error)
{
  NodeStatus node_status = NodeStatus::FAILURE;

  if (auto node = node_.lock())  // Attempt to lock the weak_ptr
  {
      RCLCPP_INFO(node->get_logger(), "[%s] Error calling drone heartbeat server ", this->name().c_str());
  }
  setOutput("error_state", "heartbeat_service_call_failed");

  return node_status;
}
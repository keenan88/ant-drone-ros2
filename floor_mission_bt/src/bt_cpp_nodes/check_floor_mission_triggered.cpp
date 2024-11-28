#include "nodes.h"

CheckFloorMissionTriggered::CheckFloorMissionTriggered(const std::string& name, const NodeConfig& conf, const RosNodeParams& params) : RosTopicSubNode<trigger_floor_mission_msg_t>(name, conf, params){}

BT::PortsList CheckFloorMissionTriggered::providedPorts()
{
    BT::PortsList ports_list = {
    BT::OutputPort<std::string>("worker_name"),
    BT::OutputPort<std::string>("drone_name"),

    BT::OutputPort<std::string>("pickup_location_name"),
    BT::OutputPort<std::string>("dropoff_location_name"),
    BT::OutputPort<std::string>("post_dropoff_location_name"),

    BT::OutputPort<float>("dropoff_orientation"),
    BT::OutputPort<float>("pickup_orientation"),
    
    BT::OutputPort<std::string>("floor_mission_state")
  };

  return ports_list;
}

NodeStatus CheckFloorMissionTriggered::onTick(const std::shared_ptr<trigger_floor_mission_msg_t>& last_msg)
{
  if(last_msg)  // is empty if no new message received, since the last tick
  {
    RCLCPP_INFO(logger(), "[%s] new message: %s to pickup %s", name().c_str(), last_msg->drone_name.c_str(), last_msg -> worker_name.c_str());

    if (last_msg->drone_name == "drone_boris")
    {
        setOutput("worker_name", last_msg->worker_name);
        setOutput("drone_name", last_msg->drone_name);
        setOutput("pickup_location_name", last_msg->pickup_location_name);
        setOutput("pickup_orientation", last_msg->pickup_orientation);
        setOutput("dropoff_location_name", last_msg->dropoff_location_name);
        setOutput("dropoff_orientation", last_msg->dropoff_orientation);
        setOutput("post_dropoff_location_name", last_msg->post_dropoff_location_name);
        setOutput("floor_mission_state", "started");
        
        return NodeStatus::SUCCESS;
    }
  }
  else
  {
    RCLCPP_INFO(logger(), "[%s] no new messages", name().c_str());
  }

  return NodeStatus::FAILURE;
}

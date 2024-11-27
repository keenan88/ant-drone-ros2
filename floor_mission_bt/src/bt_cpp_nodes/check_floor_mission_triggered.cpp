#include "nodes.h"



CheckFloorMissionTriggered::CheckFloorMissionTriggered(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_ptr) : BT::StatefulActionNode(name, config) {
  ros2_node_ptr = node_ptr;

  if (ros2_node_ptr) {
    RCLCPP_INFO(ros2_node_ptr->get_logger(), "[%s] BT.CPP node initialized", this->name().c_str());

    drone_name = ros2_node_ptr->get_namespace();
    drone_name.erase(0, 1); // Get rid of leading forwardslash from namespace
    
  }
}

void CheckFloorMissionTriggered::triggerFloorMission(const ant_fleet_interfaces::msg::TriggerFloorMission::SharedPtr msg)
{
    RCLCPP_INFO(ros2_node_ptr->get_logger(), "[%s] Received trigger message: %s", this->name().c_str(), msg->drone_name.c_str());

    // string worker_name
    // string drone_name

    // string pickup_location_name
    // string dropoff_location_name
    // float32 dropoff_orientation

    if (msg->drone_name == "drone_boris")
    {
        setOutput("worker_name", msg->worker_name);
        setOutput("drone_name", msg->drone_name);
        setOutput("pickup_location_name", msg->pickup_location_name);
        setOutput("pickup_orientation", msg->pickup_orientation);
        setOutput("dropoff_location_name", msg->dropoff_location_name);
        setOutput("dropoff_orientation", msg->dropoff_orientation);
        setOutput("post_dropoff_location_name", msg->post_dropoff_location_name);
        setOutput("floor_mission_underway", true);
        
        floor_mission_triggered = true;
    }
}

BT::NodeStatus CheckFloorMissionTriggered::onStart() {
  if (!ros2_node_ptr) {
    std::cout << "[CheckFloorMissionTriggered] ROS2 node not registered via init() method" << std::endl;

    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(ros2_node_ptr->get_logger(), "[%s] BT.CPP node running...", this->name().c_str());

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)) // Keep the last 10 messages in the queue
                       .reliable()                  // Reliable delivery
                       .transient_local();          // Ensure messages are saved for late subscribers

  subscription_ = ros2_node_ptr->create_subscription<trigger_floor_mission_msg_t>("/trigger_floor_mission", qos, std::bind(&CheckFloorMissionTriggered::triggerFloorMission, this, std::placeholders::_1));
  floor_mission_triggered = false;
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CheckFloorMissionTriggered::onRunning() {
  BT::NodeStatus node_status = BT::NodeStatus::RUNNING;

  RCLCPP_INFO(ros2_node_ptr->get_logger(), "[%s] Checking if floor mission triggered...", this->name().c_str());

  if (floor_mission_triggered) {
    subscription_ = nullptr;
    node_status = BT::NodeStatus::SUCCESS;
  }

  return node_status;
}


BT::PortsList CheckFloorMissionTriggered::providedPorts() { 
  BT::PortsList ports_list = {
    BT::OutputPort<std::string>("worker_name"),
    BT::OutputPort<std::string>("drone_name"),
    
    BT::OutputPort<std::string>("pickup_location_name"),
    BT::OutputPort<std::string>("dropoff_location_name"),
    BT::OutputPort<std::string>("post_dropoff_location_name"),
 
    BT::OutputPort<float>("dropoff_orientation"),
    BT::OutputPort<float>("pickup_orientation"),
    
    BT::OutputPort<bool>("floor_mission_underway"),
    
  };
    
  return ports_list;
  
}
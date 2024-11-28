#include "nodes.h"

InitDroneVars::InitDroneVars(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_ptr) : BT::StatefulActionNode(name, config) {
  ros2_node_ptr = node_ptr;

  if (ros2_node_ptr) {
    RCLCPP_INFO(ros2_node_ptr->get_logger(), "[%s] BT.CPP node initialized", this->name().c_str());

    drone_name = ros2_node_ptr->get_namespace();
    drone_name.erase(0, 1); // Get rid of leading forwardslash from namespace
  }
}

BT::NodeStatus InitDroneVars::onStart() {
  if (!ros2_node_ptr) {
    std::cout << "[InitDroneVars] ROS2 node not registered via init() method" << std::endl;

    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(ros2_node_ptr->get_logger(), "[%s] BT.CPP node running...", this->name().c_str());

  setOutput("drone_floor_mission_status", "IDLE");
  setOutput("drone_name", drone_name);
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus InitDroneVars::onRunning() {
  BT::NodeStatus node_status = BT::NodeStatus::RUNNING;

  node_status = BT::NodeStatus::SUCCESS;
  

  return node_status;
}


BT::PortsList InitDroneVars::providedPorts() { 
  BT::PortsList ports_list = {
      BT::OutputPort<std::string>("drone_name"),
      BT::OutputPort<std::string>("drone_floor_mission_status")
  };
    
    return ports_list;
  
}
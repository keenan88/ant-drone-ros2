#include "nodes.h"

CheckIdle::CheckIdle(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_ptr) : BT::StatefulActionNode(name, config) {
  ros2_node_ptr = node_ptr;

  if (ros2_node_ptr) {
    RCLCPP_INFO(ros2_node_ptr->get_logger(), "[%s] BT.CPP node initialized", this->name().c_str());

    subscription_ = ros2_node_ptr->create_subscription<robot_state_msg_t>("/drone_state", 10, std::bind(&CheckIdle::checkIdleCallback, this, std::placeholders::_1));

    
  }
}

void CheckIdle::checkIdleCallback(const rmf_fleet_msgs::msg::RobotState::SharedPtr msg)
{
    RCLCPP_INFO(ros2_node_ptr->get_logger(), "Received trigger message: %s", msg->name.c_str());

    // string worker_name
    // string drone_name

    // string pickup_location_name
    // string dropoff_location_name
    // float32 dropoff_orientation

    if (msg->name == "drone_boris")
    {
      int robot_mode = msg -> mode.mode;
      RCLCPP_INFO(ros2_node_ptr->get_logger(), "[%s] Got msg for drone boris %d", this->name().c_str(), robot_mode);
      if (robot_mode == 0)
      {
        RCLCPP_INFO(ros2_node_ptr->get_logger(), "[%s] drone boris confirmed idle", this->name().c_str());
        drone_idle = true;
      }
    }
}

BT::NodeStatus CheckIdle::onStart() {
  if (!ros2_node_ptr) {
    std::cout << "[FloorMissionTriggeredMsg] ROS2 node not registered via init() method" << std::endl;

    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(ros2_node_ptr->get_logger(), "[%s] Test subscription BT.CPP node running...", this->name().c_str());

  drone_idle = false;
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CheckIdle::onRunning() {
  BT::NodeStatus node_status = BT::NodeStatus::RUNNING;

  if (drone_idle) {
    node_status = BT::NodeStatus::SUCCESS;
  }

  return node_status;
}


BT::PortsList CheckIdle::providedPorts() { 
  BT::PortsList ports_list = {
    BT::OutputPort<std::string>("worker_name"),
    BT::OutputPort<std::string>("drone_name"),
    BT::OutputPort<std::string>("pickup_location_name"),
    BT::OutputPort<std::string>("dropoff_location_name"),
    BT::OutputPort<float>("dropoff_orientation"),
  };
    
  return ports_list;
  
}
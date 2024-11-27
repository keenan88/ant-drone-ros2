#include "nodes.h"

CheckPickup::CheckPickup(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_ptr) : BT::StatefulActionNode(name, config) {
  ros2_node_ptr = node_ptr;

  if (ros2_node_ptr) {
    RCLCPP_INFO(ros2_node_ptr->get_logger(), "[%s] BT.CPP node initialized", this->name().c_str());

  }
}

void CheckPickup::workerPickedUpCallback(const pickup_state_msg_t::SharedPtr msg)
{

    if (msg -> drone_name == "drone_boris")
    {

      RCLCPP_INFO(ros2_node_ptr->get_logger(), "[%s] Drone %s picked up worker %s", this->name().c_str(), msg -> drone_name.c_str(), msg -> worker_name.c_str());

      worker_picked_up = true;
      
    }
}

BT::NodeStatus CheckPickup::onStart() {
  if (!ros2_node_ptr) {
    std::cout << "[CheckPickup] ROS2 node not registered via init() method" << std::endl;

    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(ros2_node_ptr->get_logger(), "[%s] BT.CPP node running...", this->name().c_str());

  subscription_ = ros2_node_ptr->create_subscription<pickup_state_msg_t>("/worker_pickup_state", 10, std::bind(&CheckPickup::workerPickedUpCallback, this, std::placeholders::_1));

  worker_picked_up = false;
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CheckPickup::onRunning() {
  BT::NodeStatus node_status = BT::NodeStatus::RUNNING;

  if (worker_picked_up) {
    node_status = BT::NodeStatus::SUCCESS;
    subscription_ = nullptr;
  }

  return node_status;
}


BT::PortsList CheckPickup::providedPorts() { 
  BT::PortsList ports_list = {
    BT::OutputPort<std::string>("worker_name"),
    BT::OutputPort<std::string>("drone_name"),
    BT::OutputPort<std::string>("pickup_location_name"),
    BT::OutputPort<std::string>("dropoff_location_name"),
    BT::OutputPort<float>("dropoff_orientation"),
  };
    
  return ports_list;
  
}
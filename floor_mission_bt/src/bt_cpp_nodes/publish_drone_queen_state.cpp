#include "nodes.h"
#include <nlohmann/json.hpp>
#include <boost/uuid/uuid.hpp>            // for boost::uuids::uuid
#include <boost/uuid/uuid_generators.hpp> // for boost::uuids::random_generator
#include <boost/uuid/uuid_io.hpp>         // for streaming operators

PublishDroneQueenState::PublishDroneQueenState(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_ptr) : BT::StatefulActionNode(name, config) {
  ros2_node_ptr = node_ptr;

  if (ros2_node_ptr) {
    RCLCPP_INFO(ros2_node_ptr->get_logger(), "[%s] BT.CPP node initialized", this->name().c_str());

    rclcpp::QoS qos_settings(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    qos_settings.reliable();               
    qos_settings.transient_local();        
    
    publisher = ros2_node_ptr->create_publisher<StringMsg_t>("/drone_queen_state", qos_settings);
  }
}

BT::NodeStatus PublishDroneQueenState::onStart() {
  if (!ros2_node_ptr) {
    std::cout << "[PublishDroneQueenState] ROS2 node not registered via init() method" << std::endl;

    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(ros2_node_ptr->get_logger(), "[%s] BT.CPP node running...", this->name().c_str());


  auto drone_name_result = getInput<std::string>("drone_name");
  if (!drone_name_result)
  {
      RCLCPP_INFO(ros2_node_ptr->get_logger(), "[%s] Could not read blackbaord variable drone_name...", this->name().c_str());
      return BT::NodeStatus::FAILURE;
  }

  auto floor_mission_state_result = getInput<std::string>("floor_mission_state");
  if (!floor_mission_state_result)
  {
      RCLCPP_INFO(ros2_node_ptr->get_logger(), "[%s] Could not read blackbaord variable floor_mission_state...", this->name().c_str());
      return BT::NodeStatus::FAILURE;
  }

  
  std::string drone_name = drone_name_result.value();
  std::string floor_mission_state = floor_mission_state_result.value();

  StringMsg_t drone_queen_state;

  drone_queen_state.data = drone_name + "-" + floor_mission_state;

  publisher->publish(drone_queen_state);
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PublishDroneQueenState::onRunning() {
  BT::NodeStatus node_status = BT::NodeStatus::RUNNING;

  node_status = BT::NodeStatus::SUCCESS;
  

  return node_status;
}


BT::PortsList PublishDroneQueenState::providedPorts() { 
  BT::PortsList ports_list = {
      BT::InputPort<std::string>("floor_mission_state"),
      BT::InputPort<std::string>("drone_name"),
  };
    
    return ports_list;
  
}
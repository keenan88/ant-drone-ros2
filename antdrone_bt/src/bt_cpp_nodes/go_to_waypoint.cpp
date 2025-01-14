#include "nodes.h"
#include <nlohmann/json.hpp>
#include <boost/uuid/uuid.hpp>            // for boost::uuids::uuid
#include <boost/uuid/uuid_generators.hpp> // for boost::uuids::random_generator
#include <boost/uuid/uuid_io.hpp>         // for streaming operators

GoToWaypoint::GoToWaypoint(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_ptr) : BT::StatefulActionNode(name, config) {
  ros2_node_ptr = node_ptr;

  if (ros2_node_ptr) {
    RCLCPP_INFO(ros2_node_ptr->get_logger(), "[%s] BT.CPP node initialized", this->name().c_str());

    rclcpp::QoS qos_settings(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    qos_settings.reliable();               
    qos_settings.transient_local();        
    
    publisher = ros2_node_ptr->create_publisher<API_request_msg_t>("/task_api_requests", qos_settings);
  }
}

BT::NodeStatus GoToWaypoint::onStart() {
  if (!ros2_node_ptr) {
    std::cout << "[GoToWaypoint] ROS2 node not registered via init() method" << std::endl;

    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(ros2_node_ptr->get_logger(), "[%s] BT.CPP node running...", this->name().c_str());

  auto vertex_name_result = getInput<std::string>("vertex_name");
  if (!vertex_name_result)
  {
      RCLCPP_INFO(ros2_node_ptr->get_logger(), "[%s] Could not read vertex_name from blackboard", this->name().c_str());
      
      return BT::NodeStatus::FAILURE;
  }

  auto drone_name_result = getInput<std::string>("drone_name");
  if (!drone_name_result)
  {
      RCLCPP_INFO(ros2_node_ptr->get_logger(), "[%s] Could not read drone_name from blackboard", this->name().c_str());
      
      return BT::NodeStatus::FAILURE;
  }

  auto orientation_result = getInput<float>("orientation");
  if (!orientation_result)
  {
      RCLCPP_INFO(ros2_node_ptr->get_logger(), "[%s] Could not read orientation from blackboard", this->name().c_str());
      
      return BT::NodeStatus::FAILURE;
  }

  auto pickup_side_result = getInput<std::string>("pickup_side");
  if (!pickup_side_result)
  {
      RCLCPP_INFO(ros2_node_ptr->get_logger(), "[%s] Could not read pickup_side from blackboard", this->name().c_str());
      
      return BT::NodeStatus::FAILURE;
  }

  
  float orientation = orientation_result.value();
  std::string drone_name = drone_name_result.value();
  std::string vertex_name = vertex_name_result.value();
  std::string pickup_side = pickup_side_result.value();

  if (pickup_side == "L"){
    orientation += 3.14;
  }

  RCLCPP_INFO(ros2_node_ptr->get_logger(), "[%s] going to vertex %s", this->name().c_str(), vertex_name.c_str());


  // https://osrf.github.io/ros2multirobotbook/task_new.html
  nlohmann::json composed_task = {
    {"type", "robot_task_request"},
    {"robot", drone_name},
    {"fleet", "drone_fleet"},
    {"request", {
        {"category", "compose"},
        {"description", {
            {"category", "teleop"},
            {"phases", {
                {
                    {"activity", {

                        {"category", "go_to_place"},
                        {"description", {
                            {"waypoint", vertex_name},
                            {"orientation", orientation}
                        }}
                        
                    }}
                }
            }}
        }}
    }}
  };



  rmf_task_msgs::msg::ApiRequest api_request;

  boost::uuids::random_generator generator;

  boost::uuids::uuid id = generator();

  api_request.request_id = "drone-go-to-place_" + boost::uuids::to_string(id);

  api_request.json_msg = composed_task.dump();

  publisher->publish(api_request);

  RCLCPP_INFO(ros2_node_ptr->get_logger(), "[%s] Go to place: %s api request sent to RMF", this->name().c_str(), vertex_name.c_str());
  
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToWaypoint::onRunning() {
  BT::NodeStatus node_status = BT::NodeStatus::RUNNING;

  node_status = BT::NodeStatus::SUCCESS;
  

  return node_status;
}


BT::PortsList GoToWaypoint::providedPorts() { 
  BT::PortsList ports_list = {
      BT::InputPort<std::string>("vertex_name"),
      BT::InputPort<std::string>("drone_name"),
      BT::InputPort<float>("orientation"),
      BT::InputPort<std::string>("pickup_side")
  };
    
  return ports_list;

}
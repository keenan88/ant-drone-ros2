#include "nodes.h"

using namespace BT;



  CheckIfSelectedForFloorMission::CheckIfSelectedForFloorMission(const std::string& name, const NodeConfig& conf, const RosNodeParams& params): 
    RosServiceNode<ant_fleet_interfaces::srv::CheckIfSelectedForFloorMission>(name, conf, params) {}

  PortsList CheckIfSelectedForFloorMission::providedPorts() {
    return providedBasicPorts({
      InputPort<std::string>("drone_name"),
      OutputPort<std::string>("worker_name"),
      OutputPort<std::string>("pickup_location_name"),
      OutputPort<float>("pickup_orientation"),
      OutputPort<std::string>("dropoff_location_name"),
      OutputPort<float>("dropoff_orientation"),
      OutputPort<std::string>("post_dropoff_location_name"),
      OutputPort<std::string>("drone_floor_mission_status")
    });
  }



  bool CheckIfSelectedForFloorMission::setRequest(Request::SharedPtr& request) {
  
    auto drone_name_result = getInput<std::string>("drone_name");
    if (!drone_name_result)
    {
        if (auto node = node_.lock())  // Attempt to lock the weak_ptr
        {
            RCLCPP_INFO(node->get_logger(), "[%s] Could not read drone name from blackboard", this->name().c_str());
        }
        return false;
    }

    request -> robot_name = drone_name_result.value();
      
    // must return true if we are ready to send the request
    return true;
  }



  NodeStatus CheckIfSelectedForFloorMission::onResponseReceived(const Response::SharedPtr& response) {
    if(response -> is_floor_mission_triggered)
    {
      setOutput("worker_name", response->paired_robot_name);
      setOutput<std::string>("pickup_location_name", "worker_misha_pickup_point"); //response->pickup_location_name
      setOutput("pickup_orientation", response->pickup_orientation);
      setOutput("dropoff_location_name", response->dropoff_location_name);
      setOutput("dropoff_orientation", response->dropoff_orientation);
      setOutput("post_dropoff_location_name", response->post_dropoff_location_name);
      setOutput("drone_floor_mission_status", "FLOOR_MISSION");

      if (auto node = node_.lock())  // Attempt to lock the weak_ptr
      {
          RCLCPP_INFO(node->get_logger(), "[%s] Floor mission triggered", this->name().c_str());
      }

      return NodeStatus::SUCCESS;
    }

    if (auto node = node_.lock())  // Attempt to lock the weak_ptr
    {
        RCLCPP_INFO(node->get_logger(), "[%s] Floor mission not triggered", this->name().c_str());
    }

    setOutput("drone_floor_mission_status", "IDLE");

    return NodeStatus::FAILURE;
  }



  NodeStatus CheckIfSelectedForFloorMission::onFailure(ServiceNodeErrorCode error) {

    if (auto node = node_.lock())  // Attempt to lock the weak_ptr
    {
        RCLCPP_INFO(node->get_logger(), "[%s] Error calling check_if_selected_for_floor_mission server", this->name().c_str());
    }

    // RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
    return NodeStatus::FAILURE;
  }
#include "nodes.h"

using namespace BT;

CheckIfFloorMissionTriggered::CheckIfFloorMissionTriggered(
    const std::string &name, const NodeConfig &conf,
    const RosNodeParams &params)
    : RosServiceNode<ant_queen_interfaces::srv::CheckIfFloorMissionTriggered>(
          name, conf, params) {}

PortsList CheckIfFloorMissionTriggered::providedPorts() {
  return providedBasicPorts({
      InputPort<std::string>("drone_name"),
      OutputPort<std::string>("worker_name"),
      OutputPort<std::string>("pickup_location_name"),
      OutputPort<float>("pickup_orientation"),
      OutputPort<std::string>("dropoff_location_name"),
      OutputPort<float>("dropoff_orientation"),
      OutputPort<std::string>("post_dropoff_location_name"),
  });
}

bool CheckIfFloorMissionTriggered::setRequest(Request::SharedPtr &request) {

  auto drone_name_result = getInput<std::string>("drone_name");
  if (!drone_name_result) {
    if (auto node = node_.lock()) {
      RCLCPP_INFO(node->get_logger(),
                  "[%s] Could not read drone name from blackboard",
                  this->name().c_str());
    }
    return false;
  }

  request->robot_name = drone_name_result.value();

  // must return true if we are ready to send the request
  return true;
}

NodeStatus CheckIfFloorMissionTriggered::onResponseReceived(
    const Response::SharedPtr &response) {
  if (response->is_floor_mission_triggered) {
    setOutput("worker_name", response->paired_robot_name);
    setOutput<std::string>("pickup_location_name",
                           response->pickup_location_name);
    setOutput("pickup_orientation", response->pickup_orientation);
    setOutput("dropoff_location_name", response->dropoff_location_name);
    setOutput("dropoff_orientation", response->dropoff_orientation);
    setOutput("post_dropoff_location_name",
              response->post_dropoff_location_name);

    if (auto node = node_.lock()) {
      RCLCPP_INFO(node->get_logger(), "[%s] Floor mission triggered",
                  this->name().c_str());
    }

    return NodeStatus::SUCCESS;
  }

  if (auto node = node_.lock()) {
    RCLCPP_INFO(node->get_logger(), "[%s] Floor mission not triggered",
                this->name().c_str());
  }

  return NodeStatus::FAILURE;
}

NodeStatus CheckIfFloorMissionTriggered::onFailure(ServiceNodeErrorCode error) {

  if (auto node = node_.lock()) {
    RCLCPP_INFO(node->get_logger(),
                "[%s] Error calling check_if_floor_mission_triggered server",
                this->name().c_str());
  }

  // RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
  return NodeStatus::FAILURE;
}
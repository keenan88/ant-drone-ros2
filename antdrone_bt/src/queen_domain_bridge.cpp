// Copyright 2021, Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

#include "domain_bridge/component_manager.hpp"
#include "domain_bridge/domain_bridge.hpp"
#include "domain_bridge/parse_domain_bridge_yaml_config.hpp"
#include "domain_bridge/process_cmd_line_arguments.hpp"

#include "bt_datatypes.h"

int main(int argc, char **argv) {
  auto arguments = rclcpp::init_and_remove_ros_arguments(argc, argv);

  auto config_rc_pair = domain_bridge::process_cmd_line_arguments(arguments);
  if (!config_rc_pair.first || 0 != config_rc_pair.second) {
    return config_rc_pair.second;
  }
  domain_bridge::DomainBridge domain_bridge(*config_rc_pair.first);

  // Add component manager node and domain bridge to single-threaded executor
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto node = std::make_shared<domain_bridge::ComponentManager>(executor);

  uint8_t main_ros_domain_id = static_cast<uint8_t>(std::stoi(std::getenv("MAIN_ROS_DOMAIN_ID")));
  uint8_t drone_ros_domain_id = static_cast<uint8_t>(std::stoi(std::getenv("ROS_DOMAIN_ID")));

  // Add access to all of queen's servers
  domain_bridge.bridge_service<ant_queen_interfaces::srv::RegisterRobot>("/queen/register_robot", main_ros_domain_id, drone_ros_domain_id);
  domain_bridge.bridge_service<ant_queen_interfaces::srv::CheckIfFloorMissionTriggered>("/queen/check_if_floor_mission_triggered", main_ros_domain_id, drone_ros_domain_id);
  domain_bridge.bridge_service<ant_queen_interfaces::srv::LastKnownEndWaypointName>("/last_known_end_waypoint_name", main_ros_domain_id, drone_ros_domain_id);
  domain_bridge.bridge_service<ant_queen_interfaces::srv::DropoffPos>("/queen/dropoff_pos", main_ros_domain_id, drone_ros_domain_id);
  domain_bridge.bridge_service<ant_queen_interfaces::srv::MissionSuccess>("/queen/mission_success", main_ros_domain_id, drone_ros_domain_id);
  domain_bridge.bridge_service<ant_queen_interfaces::srv::ComeOut>("/queen/worker_comeout", main_ros_domain_id, drone_ros_domain_id);

  domain_bridge.add_to_executor(*executor);
  executor->add_node(node);
  executor->spin();

  rclcpp::shutdown();
  return 0;
}
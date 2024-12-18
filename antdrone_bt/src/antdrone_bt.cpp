#include <chrono>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "nodes.h"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

const std::string default_bt_xml_file = "/home/humble_ws/src/antdrone_bt/behavior_trees/floor_mission.xml";

class FloorMissionNode : public rclcpp::Node {
public:
  FloorMissionNode() : Node("bt") {
    this->declare_parameter<std::string>("DRONE_NAME", "");
    this->declare_parameter<std::string>("tree_xml_file", default_bt_xml_file);
    tree_xml_file_ = this->get_parameter("tree_xml_file").as_string();

    RCLCPP_WARN(this->get_logger(), "xml file: %s", tree_xml_file_.c_str());
  }

  void create_behavior_tree() {
    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<GoToWaypoint>("GoToWaypoint", shared_from_this());
    factory.registerNodeType<RegisterDrone>("RegisterDrone", BT::RosNodeParams(shared_from_this(), "/queen/register_robot"));
    factory.registerNodeType<CheckGoToWaypointSuccess>("CheckGoToWaypointSuccess", BT::RosNodeParams(shared_from_this(), "/last_known_end_waypoint_name"));
    factory.registerNodeType<CheckRMFClientIdle>("CheckRMFClientIdle", BT::RosNodeParams(shared_from_this(), "/check_rmf_client_idle"));
    factory.registerNodeType<CheckIfFloorMissionTriggered>("CheckIfFloorMissionTriggered", BT::RosNodeParams(shared_from_this(), "/queen/check_if_floor_mission_triggered"));
    factory.registerNodeType<CheckHeartbeat>("CheckHeartbeat", BT::RosNodeParams(shared_from_this(), "/mission_heartbeat"));
    factory.registerNodeType<SuspendReleaseRMFPathing>("SuspendReleaseRMFPathing", BT::RosNodeParams(shared_from_this(), "/suspend_release_rmf_pathing"));
    factory.registerNodeType<ClearCostmap>("ClearLocalCostmap", BT::RosNodeParams(shared_from_this(), "/local_costmap/clear_entirely_local_costmap"));
    factory.registerNodeType<ClearCostmap>("ClearGlobalCostmap", BT::RosNodeParams(shared_from_this(), "/global_costmap/clear_entirely_global_costmap"));
    factory.registerNodeType<TriggerWorkerComeout>("TriggerWorkerComeout", BT::RosNodeParams(shared_from_this(), "/queen/worker_comeout"));
    factory.registerNodeType<CheckComeOutComplete>("CheckComeOutComplete", BT::RosNodeParams(shared_from_this(), "/queen/worker_comeout"));
    factory.registerNodeType<PickupWorker>("PickupWorker", BT::RosNodeParams(shared_from_this(), "/ATTACHLINK"));
    factory.registerNodeType<UpdateFootprint>("UpdateFootprint", shared_from_this());
    factory.registerNodeType<SetCostmapParams>("SetLocalCostmapParams", BT::RosNodeParams(shared_from_this(), "/local_costmap/local_costmap/set_parameters"));
    factory.registerNodeType<SetCostmapParams>("SetGlobalCostmapParams", BT::RosNodeParams(shared_from_this(), "/global_costmap/global_costmap/set_parameters"));
    factory.registerNodeType<LowerWorker>("LowerWorker", BT::RosNodeParams(shared_from_this(), "/DETACHLINK"));
    factory.registerNodeType<SendDropoffPosition>("SendDropoffPosition", BT::RosNodeParams(shared_from_this(), "/queen/dropoff_pos"));
    factory.registerNodeType<SucceedFloorMission>("SucceedFloorMission", BT::RosNodeParams(shared_from_this(), "/queen/mission_success"));

    auto blackboard = BT::Blackboard::create();
    tree_ = factory.createTreeFromFile(tree_xml_file_, blackboard);

    RCLCPP_INFO(this->get_logger(), "All BT.CPP nodes registered");
  }

  void execute() {
    create_behavior_tree();

    const auto timer_period = 250ms;
    timer_ = this->create_wall_timer(timer_period, std::bind(&FloorMissionNode::update_behavior_tree, this));

    rclcpp::spin(shared_from_this());
    rclcpp::shutdown();
  }

  void update_behavior_tree() {
    BT::NodeStatus tree_status = tree_.tickOnce();
    if (tree_status == BT::NodeStatus::RUNNING) {
      return;
    }
    if (tree_status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Finished with status SUCCESS");
      timer_->cancel();
    } else if (tree_status == BT::NodeStatus::FAILURE) {
      RCLCPP_INFO(this->get_logger(), "Finished with status FAILURE");
      timer_->cancel();
    }
  }

  std::string tree_xml_file_;

  rclcpp::TimerBase::SharedPtr timer_;
  BT::Tree tree_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FloorMissionNode>();

  node->execute();

  return 0;
}

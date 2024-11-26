#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "behaviortree_cpp/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_msgs/msg/string.hpp"
#include "rmf_task_msgs/msg/api_request.hpp"
#include "ant_fleet_interfaces/msg/trigger_floor_mission.hpp"

class CheckFloorMissionTriggered : public BT::StatefulActionNode
{
  public:
    using trigger_floor_mission_msg_t = ant_fleet_interfaces::msg::TriggerFloorMission;
    rclcpp::Subscription<trigger_floor_mission_msg_t>::SharedPtr subscription_;
    bool floor_mission_triggered = false;
    rclcpp::Node::SharedPtr ros2_node_ptr;

    CheckFloorMissionTriggered(const std::string &name, const BT::NodeConfig &config, rclcpp::Node::SharedPtr node_ptr);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override{};
    static BT::PortsList providedPorts();

    void triggerFloorMission(const ant_fleet_interfaces::msg::TriggerFloorMission::SharedPtr msg);
};

class GoToPlace : public BT::StatefulActionNode
{
  public:
    using API_request_msg_t = rmf_task_msgs::msg::ApiRequest;
    using StringMsg_t = std_msgs::msg::String;
    rclcpp::Node::SharedPtr ros2_node_ptr;
    rclcpp::Publisher<API_request_msg_t>::SharedPtr publisher;
    bool done_flag;

    GoToPlace(const std::string &name, const BT::NodeConfig &config, rclcpp::Node::SharedPtr node_ptr);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override{};
    static BT::PortsList providedPorts();

};


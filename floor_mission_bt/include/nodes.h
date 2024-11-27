#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "behaviortree_cpp/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_msgs/msg/string.hpp"
#include "rmf_task_msgs/msg/api_request.hpp"
#include "rmf_fleet_msgs/msg/robot_state.hpp"
#include "ant_fleet_interfaces/msg/trigger_floor_mission.hpp"
#include "ant_fleet_interfaces/msg/worker_pickup_state.hpp"
#include "ant_fleet_interfaces/srv/suspend_rmf_pathing.hpp"

#include "linkattacher_msgs/srv/attach_link.hpp"
#include "linkattacher_msgs/srv/detach_link.hpp"

#include <behaviortree_ros2/bt_service_node.hpp>

#include "ant_fleet_interfaces/srv/request_worker_pickup.hpp"
#include "linkattacher_msgs/srv/attach_link.hpp"

using namespace BT;


class CheckFloorMissionTriggered : public BT::StatefulActionNode
{
  public:
    using trigger_floor_mission_msg_t = ant_fleet_interfaces::msg::TriggerFloorMission;
    rclcpp::Subscription<trigger_floor_mission_msg_t>::SharedPtr subscription_;
    bool floor_mission_triggered = false;
    std::string drone_name;
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

class CheckIdle : public BT::StatefulActionNode
{
  public:
    using robot_state_msg_t = rmf_fleet_msgs::msg::RobotState;
    rclcpp::Subscription<robot_state_msg_t>::SharedPtr subscription_;
    bool drone_idle;
    rclcpp::Node::SharedPtr ros2_node_ptr;

    CheckIdle(const std::string &name, const BT::NodeConfig &config, rclcpp::Node::SharedPtr node_ptr);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override{};
    static BT::PortsList providedPorts();

    void checkIdleCallback(const rmf_fleet_msgs::msg::RobotState::SharedPtr msg);
};




class CheckPickup : public BT::StatefulActionNode
{
  public:
    using pickup_state_msg_t = ant_fleet_interfaces::msg::WorkerPickupState;
    rclcpp::Subscription<pickup_state_msg_t>::SharedPtr subscription_;
    bool worker_picked_up;
    rclcpp::Node::SharedPtr ros2_node_ptr;

    CheckPickup(const std::string &name, const BT::NodeConfig &config, rclcpp::Node::SharedPtr node_ptr);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override{};
    static BT::PortsList providedPorts();

    void workerPickedUpCallback(const pickup_state_msg_t::SharedPtr msg);
};

using SendPickupCmd_srv_t = linkattacher_msgs::srv::AttachLink;
class SendPickupCmd: public RosServiceNode<SendPickupCmd_srv_t>
{
  public:

  SendPickupCmd(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);
  static PortsList providedPorts();
  bool setRequest(Request::SharedPtr& request) override;
  NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
  virtual NodeStatus onFailure(ServiceNodeErrorCode error) override;
};


using SuspendRMFPathing_srv_t = ant_fleet_interfaces::srv::SuspendRMFPathing;
class SendSuspendRMFPathing: public RosServiceNode<SuspendRMFPathing_srv_t>
{
  public:

  SendSuspendRMFPathing(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);
  static PortsList providedPorts();
  bool setRequest(Request::SharedPtr& request) override;
  NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
  virtual NodeStatus onFailure(ServiceNodeErrorCode error) override;
};


class SendReleaseRMFPathing: public RosServiceNode<SuspendRMFPathing_srv_t>
{
  public:

  SendReleaseRMFPathing(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);
  static PortsList providedPorts();
  bool setRequest(Request::SharedPtr& request) override;
  NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
  virtual NodeStatus onFailure(ServiceNodeErrorCode error) override;
};
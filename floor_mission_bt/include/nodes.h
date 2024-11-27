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
#include "behaviortree_ros2/bt_topic_sub_node.hpp"

#include "ant_fleet_interfaces/srv/request_worker_pickup.hpp"
#include "ant_fleet_interfaces/srv/check_drone_idle.hpp"
#include "linkattacher_msgs/srv/attach_link.hpp"

using namespace BT;

using trigger_floor_mission_msg_t = ant_fleet_interfaces::msg::TriggerFloorMission;
class CheckFloorMissionTriggered : public RosTopicSubNode<trigger_floor_mission_msg_t>
{
public:
  std::string drone_name;
  CheckFloorMissionTriggered(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);

  static BT::PortsList providedPorts();

  NodeStatus onTick(const std::shared_ptr<trigger_floor_mission_msg_t>& last_msg) override;
};

// class CheckFloorMissionTriggered : public BT::StatefulActionNode
// {
//   public:
//     
//     rclcpp::Subscription<trigger_floor_mission_msg_t>::SharedPtr subscription_;
//     bool floor_mission_triggered = false;
//     std::string drone_name;
//     rclcpp::Node::SharedPtr ros2_node_ptr;

//     CheckFloorMissionTriggered(const std::string &name, const BT::NodeConfig &config, rclcpp::Node::SharedPtr node_ptr);
//     BT::NodeStatus onStart() override;
//     BT::NodeStatus onRunning() override;
//     void onHalted() override{};
//     static BT::PortsList providedPorts();

//     void triggerFloorMission(const ant_fleet_interfaces::msg::TriggerFloorMission::SharedPtr msg);
// };

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

using CheckDroneIdle_srv_t = ant_fleet_interfaces::srv::CheckDroneIdle;
class CheckIdle: public RosServiceNode<CheckDroneIdle_srv_t>
{
  public:

  CheckIdle(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);
  static PortsList providedPorts();
  bool setRequest(Request::SharedPtr& request) override;
  NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
  virtual NodeStatus onFailure(ServiceNodeErrorCode error) override;
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
class PickupWorker: public RosServiceNode<SendPickupCmd_srv_t>
{
  public:

  PickupWorker(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);
  static PortsList providedPorts();
  bool setRequest(Request::SharedPtr& request) override;
  NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
  virtual NodeStatus onFailure(ServiceNodeErrorCode error) override;
};


using SuspendRMFPathing_srv_t = ant_fleet_interfaces::srv::SuspendRMFPathing;
class SuspendRMFPathing: public RosServiceNode<SuspendRMFPathing_srv_t>
{
  public:

  SuspendRMFPathing(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);
  static PortsList providedPorts();
  bool setRequest(Request::SharedPtr& request) override;
  NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
  virtual NodeStatus onFailure(ServiceNodeErrorCode error) override;
};


class ReleaseRMFPathing: public RosServiceNode<SuspendRMFPathing_srv_t>
{
  public:

  ReleaseRMFPathing(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);
  static PortsList providedPorts();
  bool setRequest(Request::SharedPtr& request) override;
  NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
  virtual NodeStatus onFailure(ServiceNodeErrorCode error) override;
};

using SendLowerCmd_srv_t = linkattacher_msgs::srv::DetachLink;
class LowerWorker: public RosServiceNode<SendLowerCmd_srv_t>
{
  public:

  LowerWorker(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);
  static PortsList providedPorts();
  bool setRequest(Request::SharedPtr& request) override;
  NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
  virtual NodeStatus onFailure(ServiceNodeErrorCode error) override;
};

class PublishDroneQueenState : public BT::StatefulActionNode
{
  public:
    using StringMsg_t = std_msgs::msg::String;
    rclcpp::Node::SharedPtr ros2_node_ptr;
    rclcpp::Publisher<StringMsg_t>::SharedPtr publisher;

    PublishDroneQueenState(const std::string &name, const BT::NodeConfig &config, rclcpp::Node::SharedPtr node_ptr);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override{};
    static BT::PortsList providedPorts();

};

class InitDroneVars : public BT::StatefulActionNode
{
  public:

    std::string drone_name;

    rclcpp::Node::SharedPtr ros2_node_ptr;

    InitDroneVars(const std::string &name, const BT::NodeConfig &config, rclcpp::Node::SharedPtr node_ptr);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override{};
    static BT::PortsList providedPorts();

};



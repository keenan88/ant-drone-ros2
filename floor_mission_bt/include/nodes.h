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
#include "ant_fleet_interfaces/msg/worker_pickup_state.hpp"
#include "ant_fleet_interfaces/srv/suspend_rmf_pathing.hpp"

#include "linkattacher_msgs/srv/attach_link.hpp"
#include "linkattacher_msgs/srv/detach_link.hpp"

#include <behaviortree_ros2/bt_service_node.hpp>
#include "behaviortree_ros2/bt_topic_sub_node.hpp"

#include "ant_fleet_interfaces/srv/mission_success.hpp"
#include "ant_fleet_interfaces/srv/check_if_selected_for_floor_mission.hpp"
#include "ant_fleet_interfaces/srv/mission_heartbeat.hpp"
#include "ant_fleet_interfaces/srv/request_worker_pickup.hpp"
#include "ant_fleet_interfaces/srv/check_drone_idle.hpp"
#include "linkattacher_msgs/srv/attach_link.hpp"

using namespace BT;



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

class CheckIfSelectedForFloorMission: public RosServiceNode<ant_fleet_interfaces::srv::CheckIfSelectedForFloorMission>
{
  public:

  CheckIfSelectedForFloorMission(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);
  static PortsList providedPorts();
  bool setRequest(Request::SharedPtr& request) override;
  NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
  virtual NodeStatus onFailure(ServiceNodeErrorCode error) override;
};

class SendHeartbeatToQueen: public RosServiceNode<ant_fleet_interfaces::srv::MissionHeartbeat>
{
  public:

  int64_t last_heartbeat_time_s = -1;
  int64_t heartbeat_timeout_s = 10;

  SendHeartbeatToQueen(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);
  static PortsList providedPorts();
  bool setRequest(Request::SharedPtr& request) override;
  NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
  virtual NodeStatus onFailure(ServiceNodeErrorCode error) override;
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


class SendFloorMissionSuccess: public RosServiceNode<ant_fleet_interfaces::srv::MissionSuccess>
{
  public:

  SendFloorMissionSuccess(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);
  static PortsList providedPorts();
  bool setRequest(Request::SharedPtr& request) override;
  NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
  virtual NodeStatus onFailure(ServiceNodeErrorCode error) override;
};



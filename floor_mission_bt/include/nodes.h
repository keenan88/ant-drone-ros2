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
#include "ant_fleet_interfaces/srv/mission_heartbeat_srv.hpp"
#include "ant_fleet_interfaces/srv/request_worker_pickup.hpp"
#include "ant_fleet_interfaces/srv/check_drone_idle.hpp"
#include "ant_fleet_interfaces/srv/last_known_end_waypoint_name.hpp"
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



class CheckHeartbeat: public RosServiceNode<ant_fleet_interfaces::srv::MissionHeartbeatSrv>
{
  public:

  CheckHeartbeat(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);
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



using namespace BT;

class CheckIfSelectedForFloorMission: public RosServiceNode<ant_fleet_interfaces::srv::CheckIfSelectedForFloorMission>
{
  public:

  CheckIfSelectedForFloorMission(const std::string& name, const NodeConfig& conf, const RosNodeParams& params): 
    RosServiceNode<ant_fleet_interfaces::srv::CheckIfSelectedForFloorMission>(name, conf, params) {}

  static PortsList providedPorts() {
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



  bool setRequest(Request::SharedPtr& request) override {
  
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



  NodeStatus onResponseReceived(const Response::SharedPtr& response) override {
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



  virtual NodeStatus onFailure(ServiceNodeErrorCode error) override {

    if (auto node = node_.lock())  // Attempt to lock the weak_ptr
    {
        RCLCPP_INFO(node->get_logger(), "[%s] Error calling check_if_selected_for_floor_mission server", this->name().c_str());
    }

    // RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
    return NodeStatus::FAILURE;
  }
};











class CheckGoToPlaceSuccess: public RosServiceNode<ant_fleet_interfaces::srv::LastKnownEndWaypointName>
{
  public:

  std::string desired_vertex_name;

  CheckGoToPlaceSuccess(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)  : 
    RosServiceNode<ant_fleet_interfaces::srv::LastKnownEndWaypointName>(name, conf, params) {};


  static PortsList providedPorts(){

    return providedBasicPorts ({
      BT::InputPort<std::string>("vertex_name"),
      BT::OutputPort<std::string>("error_state")
    });

  }


  bool setRequest(Request::SharedPtr& request) override {

    getInput("vertex_name", desired_vertex_name);

    if (auto node = node_.lock())  // Attempt to lock the weak_ptr
    {
        RCLCPP_INFO(node->get_logger(), "[%s] From blackboard, desired_vertex_name = %s ", this->name().c_str(), desired_vertex_name.c_str());
    }
    

    // desired_vertex_name = desired_vertex_name_result.value();
      
    // must return true if we are ready to send the request
    return true;
  }


  NodeStatus onResponseReceived(const Response::SharedPtr& response) override {
    NodeStatus node_status = NodeStatus::FAILURE;

    if(response -> last_known_waypoint_name == desired_vertex_name)
    {
      node_status = NodeStatus::SUCCESS;
      if (auto node = node_.lock())  // Attempt to lock the weak_ptr
      {
          RCLCPP_INFO(node->get_logger(), "[%s] Drone has reached desired vertex %s ", this->name().c_str(), desired_vertex_name.c_str());
      }
    }
    else
    {
      if (auto node = node_.lock())  // Attempt to lock the weak_ptr
      {
          RCLCPP_INFO(node->get_logger(), "[%s] Last known end vertex: %s, desired: %s ", this->name().c_str(), response -> last_known_waypoint_name.c_str(), desired_vertex_name.c_str());
      }
    }

    return node_status;
  }


  virtual NodeStatus onFailure(ServiceNodeErrorCode error) override {
    NodeStatus node_status = NodeStatus::FAILURE;

    if (auto node = node_.lock())  // Attempt to lock the weak_ptr
    {
        RCLCPP_INFO(node->get_logger(), "[%s] Error calling drone last_known_end_waypoint_name server ", this->name().c_str());
    }
    setOutput("error_state", "last_known_end_waypoint_name_call_failed");

    return node_status;
  }


};
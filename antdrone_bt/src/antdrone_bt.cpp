#include <chrono>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "nodes.h"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

const std::string default_bt_xml_file = "/home/humble_ws/src/antdrone_bt/behavior_trees/floor_mission.xml";

class FloorMissionNode : public rclcpp::Node
{
  public:
    FloorMissionNode() : Node("bt")
    {
        this->declare_parameter<std::string>("DRONE_NAME", "");
        this->declare_parameter<std::string>("tree_xml_file", default_bt_xml_file);
        tree_xml_file_ = this->get_parameter("tree_xml_file").as_string();
        

        RCLCPP_WARN(this->get_logger(), "xml file: %s", tree_xml_file_.c_str());
    }

    void create_behavior_tree()
    {
        BT::BehaviorTreeFactory factory;

        factory.registerNodeType<GoToWaypoint>("GoToWaypoint", shared_from_this());

        auto register_robot_params = BT::RosNodeParams(shared_from_this(), "/queen/register_robot");
        register_robot_params.wait_for_server_timeout = std::chrono::milliseconds(5000);
        factory.registerNodeType<RegisterDrone>("RegisterDrone", register_robot_params);

        auto check_go_to_place_success_params = BT::RosNodeParams(shared_from_this(), "/last_known_end_waypoint_name");
        check_go_to_place_success_params.wait_for_server_timeout = std::chrono::milliseconds(5000);
        factory.registerNodeType<CheckGoToWaypointSuccess>("CheckGoToWaypointSuccess", check_go_to_place_success_params);

        auto check_idle_params = BT::RosNodeParams(shared_from_this(), "/check_rmf_client_idle");
        check_idle_params.wait_for_server_timeout = std::chrono::milliseconds(5000);
        factory.registerNodeType<CheckRMFClientIdle>("CheckRMFClientIdle", check_idle_params);

        auto check_selected_for_floor_mission_params = BT::RosNodeParams(shared_from_this(), "/queen/check_if_floor_mission_triggered"); 
        check_selected_for_floor_mission_params.wait_for_server_timeout = std::chrono::milliseconds(5000);
        factory.registerNodeType<CheckIfFloorMissionTriggered>("CheckIfFloorMissionTriggered", check_selected_for_floor_mission_params);

        auto check_heartbeat_params = BT::RosNodeParams(shared_from_this(), "/mission_heartbeat");
        check_heartbeat_params.wait_for_server_timeout = std::chrono::milliseconds(5000);
        factory.registerNodeType<CheckHeartbeat>("CheckHeartbeat", check_heartbeat_params);

        auto rmf_path_suspend_node_params = BT::RosNodeParams(shared_from_this(), "/suspend_release_rmf_pathing");
        rmf_path_suspend_node_params.wait_for_server_timeout = std::chrono::milliseconds(5000);
        factory.registerNodeType<SuspendReleaseRMFPathing>("SuspendReleaseRMFPathing", rmf_path_suspend_node_params);

        auto clear_local_costmap_params = BT::RosNodeParams(shared_from_this(), "/local_costmap/clear_entirely_local_costmap");
        clear_local_costmap_params.wait_for_server_timeout = std::chrono::milliseconds(5000);
        factory.registerNodeType<ClearCostmap>("ClearLocalCostmap", clear_local_costmap_params);

        auto clear_global_costmap_params = BT::RosNodeParams(shared_from_this(), "/global_costmap/clear_entirely_global_costmap");
        clear_global_costmap_params.wait_for_server_timeout = std::chrono::milliseconds(5000);
        factory.registerNodeType<ClearCostmap>("ClearGlobalCostmap", clear_global_costmap_params);

        auto send_comeout_trigger_params = BT::RosNodeParams(shared_from_this(), "/queen/worker_comeout");
        send_comeout_trigger_params.wait_for_server_timeout = std::chrono::milliseconds(5000);
        factory.registerNodeType<TriggerWorkerComeout>("TriggerWorkerComeout", send_comeout_trigger_params);

        auto check_comeout_complete_params = BT::RosNodeParams(shared_from_this(), "/queen/worker_comeout");
        check_comeout_complete_params.wait_for_server_timeout = std::chrono::milliseconds(5000);
        factory.registerNodeType<CheckComeOutComplete>("CheckComeOutComplete", check_comeout_complete_params);
        
        auto pickup_params = BT::RosNodeParams(shared_from_this(), "/ATTACHLINK");
        pickup_params.wait_for_server_timeout = std::chrono::milliseconds(5000);
        factory.registerNodeType<LowerPickupWorker>("PickupWorker", pickup_params);

        factory.registerNodeType<UpdateFootprint>("UpdateFootprint", shared_from_this());

        auto set_local_costmap_params_node = BT::RosNodeParams(shared_from_this(), "/local_costmap/local_costmap/set_parameters");
        set_local_costmap_params_node.wait_for_server_timeout = std::chrono::milliseconds(5000);
        factory.registerNodeType<SetCostmapParams>("SetLocalCostmapParams", set_local_costmap_params_node);

        auto set_global_costmap_params_node = BT::RosNodeParams(shared_from_this(), "/global_costmap/global_costmap/set_parameters");
        set_global_costmap_params_node.wait_for_server_timeout = std::chrono::milliseconds(5000);
        factory.registerNodeType<SetCostmapParams>("SetGlobalCostmapParams", set_global_costmap_params_node);

        auto dropoff_params = BT::RosNodeParams(shared_from_this(), "/DETACHLINK");
        dropoff_params.wait_for_server_timeout = std::chrono::milliseconds(5000);
        factory.registerNodeType<LowerPickupWorker>("LowerWorker", dropoff_params);

        auto send_dropoff_position_params = BT::RosNodeParams(shared_from_this(), "/queen/dropoff_pos");
        send_dropoff_position_params.wait_for_server_timeout = std::chrono::milliseconds(5000);
        factory.registerNodeType<SendDropoffPosition>("SendDropoffPosition", send_dropoff_position_params);

        auto send_floor_mission_success_params = BT::RosNodeParams(shared_from_this(), "/queen/mission_success");
        send_floor_mission_success_params.wait_for_server_timeout = std::chrono::milliseconds(5000);
        factory.registerNodeType<SucceedFloorMission>("SucceedFloorMission", send_floor_mission_success_params);
        
        auto blackboard = BT::Blackboard::create();
        tree_ = factory.createTreeFromFile(tree_xml_file_, blackboard);

        RCLCPP_INFO(this->get_logger(), "All BT.CPP nodes registered");
    }

    void execute()
    {        
        create_behavior_tree();

        const auto timer_period = 250ms;
        timer_ = this->create_wall_timer(timer_period, std::bind(&FloorMissionNode::update_behavior_tree, this));

        rclcpp::spin(shared_from_this());
        rclcpp::shutdown();
    }

    void update_behavior_tree()
    {
        BT::NodeStatus tree_status = tree_.tickOnce();
        if (tree_status == BT::NodeStatus::RUNNING)
        {
            return;
        }
        if (tree_status == BT::NodeStatus::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Finished with status SUCCESS");
            timer_ -> cancel();
        }
        else if (tree_status == BT::NodeStatus::FAILURE)
        {
            RCLCPP_INFO(this->get_logger(), "Finished with status FAILURE");
            timer_ -> cancel();
        }
        
    }

    std::string tree_xml_file_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    BT::Tree tree_;
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FloorMissionNode>();

    node -> execute();

    return 0;
}

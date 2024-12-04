#include <chrono>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "nodes.h"
#include "rclcpp/rclcpp.hpp"

// Purpose-built Bt.CPP nodes for calling ros2 actions and services: https://www.behaviortree.dev/docs/ros2_integration/


using namespace std::chrono_literals;

const std::string default_bt_xml_file = "/home/humble_ws/src/floor_mission_bt/behavior_trees/floor_mission.xml";

class FloorMissionNode : public rclcpp::Node
{
  public:
    FloorMissionNode() : Node("bt")
    {
        this->declare_parameter<std::string>("tree_xml_file", default_bt_xml_file);
        tree_xml_file_ = this->get_parameter("tree_xml_file").as_string();
        RCLCPP_WARN(this->get_logger(), "xml file: %s", tree_xml_file_.c_str());
    }

    void create_behavior_tree()
    {
        BT::BehaviorTreeFactory factory;

        factory.registerNodeType<InitDroneVars>("InitDroneVars", shared_from_this());


        factory.registerNodeType<GoToPlace>("GoToPlace", shared_from_this());

        auto register_robot_params = BT::RosNodeParams(shared_from_this(), "/queen/register_robot");
        register_robot_params.wait_for_server_timeout = std::chrono::milliseconds(5000);
        factory.registerNodeType<RegisterDrone>("RegisterDrone", register_robot_params);

        auto check_go_to_place_success_params = BT::RosNodeParams(shared_from_this(), "/drone_boris/last_known_end_waypoint_name");
        check_go_to_place_success_params.wait_for_server_timeout = std::chrono::milliseconds(5000);
        factory.registerNodeType<CheckGoToPlaceSuccess>("CheckGoToPlaceSuccess", check_go_to_place_success_params);


        auto check_idle_params = BT::RosNodeParams(shared_from_this(), "/drone_boris/check_drone_idle");
        check_idle_params.wait_for_server_timeout = std::chrono::milliseconds(5000);
        factory.registerNodeType<CheckIdle>("CheckIdle", check_idle_params);

        auto check_selected_for_floor_mission_params = BT::RosNodeParams(shared_from_this(), "/queen/check_if_selected_for_floor_mission"); 
        check_selected_for_floor_mission_params.wait_for_server_timeout = std::chrono::milliseconds(5000);
        factory.registerNodeType<CheckIfSelectedForFloorMission>("CheckIfSelectedForFloorMission", check_selected_for_floor_mission_params);

        auto check_heartbeat_params = BT::RosNodeParams(shared_from_this(), "/drone_boris/mission_heartbeat");
        check_heartbeat_params.wait_for_server_timeout = std::chrono::milliseconds(5000);
        factory.registerNodeType<CheckHeartbeat>("CheckHeartbeat", check_heartbeat_params);


        auto rmf_path_suspend_node_params = BT::RosNodeParams(shared_from_this(), "/drone_boris/suspend_rmf_pathing");
        rmf_path_suspend_node_params.wait_for_server_timeout = std::chrono::milliseconds(5000);
        factory.registerNodeType<SuspendRMFPathing>("SuspendRMFPathing", rmf_path_suspend_node_params);

        
        auto pickup_params = BT::RosNodeParams(shared_from_this(), "/ATTACHLINK");
        pickup_params.wait_for_server_timeout = std::chrono::milliseconds(5000);
        factory.registerNodeType<PickupWorker>("PickupWorker", pickup_params);


        auto rmf_path_release_node_params = BT::RosNodeParams(shared_from_this(), "/drone_boris/suspend_rmf_pathing");
        rmf_path_release_node_params.wait_for_server_timeout = std::chrono::milliseconds(5000);
        factory.registerNodeType<ReleaseRMFPathing>("ReleaseRMFPathing", rmf_path_release_node_params);


        auto dropoff_params = BT::RosNodeParams(shared_from_this(), "/DETACHLINK");
        dropoff_params.wait_for_server_timeout = std::chrono::milliseconds(5000);
        factory.registerNodeType<LowerWorker>("LowerWorker", dropoff_params);

        auto send_dropoff_position_params = BT::RosNodeParams(shared_from_this(), "/queen/dropped_off_position");
        send_dropoff_position_params.wait_for_server_timeout = std::chrono::milliseconds(5000);
        factory.registerNodeType<SendDropoffPosition>("SendDropoffPosition", send_dropoff_position_params);

        auto send_floor_mission_success_params = BT::RosNodeParams(shared_from_this(), "/queen/floor_mission_success");
        send_floor_mission_success_params.wait_for_server_timeout = std::chrono::milliseconds(5000);
        factory.registerNodeType<SendFloorMissionSuccess>("SendFloorMissionSuccess", send_floor_mission_success_params);

        auto moveout_params = BT::RosNodeParams(shared_from_this(), "/drone_boris/moveout");
        moveout_params.wait_for_server_timeout = std::chrono::milliseconds(5000);
        moveout_params.server_timeout = std::chrono::milliseconds(10000);
        factory.registerNodeType<MoveOut>("MoveOut", moveout_params);

        
        auto blackboard = BT::Blackboard::create();
        tree_ = factory.createTreeFromFile(tree_xml_file_, blackboard);

        RCLCPP_INFO(this->get_logger(), "All BT.CPP nodes registered");
    }

    void execute()
    {        
        create_behavior_tree();

        const auto timer_period = 500ms;
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

#include <chrono>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "nodes.h"
#include "rclcpp/rclcpp.hpp"




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

        factory.registerNodeType<GoToPlace>("GoToPlace", shared_from_this());
        factory.registerNodeType<CheckFloorMissionTriggered>("CheckFloorMissionTriggered", shared_from_this());
        
        
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

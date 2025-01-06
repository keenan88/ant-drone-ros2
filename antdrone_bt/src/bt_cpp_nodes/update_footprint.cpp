#include "nodes.h"
#include "geometry_msgs/msg/point32.hpp"

UpdateFootprint::UpdateFootprint(const std::string &name, const BT::NodeConfig &config, rclcpp::Node::SharedPtr node_ptr) : BT::StatefulActionNode(name, config)
{
    ros2_node_ptr = node_ptr;
    if (ros2_node_ptr)
    {
        RCLCPP_INFO(ros2_node_ptr->get_logger(), "[%s] BT.CPP node initialized", this->name().c_str());

        local_footprint_publisher = ros2_node_ptr->create_publisher<FootprintMsg_t>("local_costmap/footprint", 10);
        global_footprint_publisher = ros2_node_ptr->create_publisher<FootprintMsg_t>("global_costmap/footprint", 10);
    }
}

BT::NodeStatus UpdateFootprint::onStart()
{
    if (!ros2_node_ptr)
    {
        std::cout << "[UpdateFootprint] ROS2 node not registered via init() method" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus UpdateFootprint::onRunning()
{
    BT::NodeStatus node_status = BT::NodeStatus::FAILURE;

    auto is_carrying_worker_result = getInput<bool>("is_carrying_worker");
    

    if (!is_carrying_worker_result)
    {
        std::cerr << "Couldn't get is_carrying_worker!" << std::endl;
    }    
    else
    {
        auto polygon = FootprintMsg_t();

        bool is_carrying_worker = bool(is_carrying_worker_result.value());

        if(is_carrying_worker)
        {   
            // Set drone's footprint to worker's footprint, rotated 90 degrees
            polygon.points.push_back(geometry_msgs::msg::Point32());
            polygon.points[0].x = 0.4;
            polygon.points[0].y = 0.5;
            polygon.points[0].z = 0.0;
            polygon.points.push_back(geometry_msgs::msg::Point32());
            polygon.points[1].x = 0.4;
            polygon.points[1].y = -0.5;
            polygon.points[1].z = 0.0;
            polygon.points.push_back(geometry_msgs::msg::Point32());
            polygon.points[2].x = -0.4;
            polygon.points[2].y = -0.5;
            polygon.points[2].z = 0.0;
            polygon.points.push_back(geometry_msgs::msg::Point32());
            polygon.points[3].x = -0.4;
            polygon.points[3].y = 0.5;
            polygon.points[3].z = 0.0;
        }
        else
        {
            polygon.points.push_back(geometry_msgs::msg::Point32());
            polygon.points[0].x = 0.41;
            polygon.points[0].y = 0.25;
            polygon.points[0].z = 0.0;
            polygon.points.push_back(geometry_msgs::msg::Point32());
            polygon.points[1].x = 0.41;
            polygon.points[1].y = -0.25;
            polygon.points[1].z = 0.0;
            polygon.points.push_back(geometry_msgs::msg::Point32());
            polygon.points[2].x = -0.41;
            polygon.points[2].y = -0.25;
            polygon.points[2].z = 0.0;
            polygon.points.push_back(geometry_msgs::msg::Point32());
            polygon.points[3].x = -0.41;
            polygon.points[3].y = 0.25;
            polygon.points[3].z = 0.0;
        }

        local_footprint_publisher->publish(polygon);
        global_footprint_publisher->publish(polygon);

        node_status = BT::NodeStatus::SUCCESS;
    }

    return node_status;
}

BT::PortsList UpdateFootprint::providedPorts()
{
    BT::PortsList ports_list = {
        BT::InputPort<bool>("is_carrying_worker")
    };
    
    return ports_list;
}
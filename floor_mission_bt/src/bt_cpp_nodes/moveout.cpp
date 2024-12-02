#include "nodes.h"


MoveOut::MoveOut(const std::string& name, const NodeConfig& conf, const RosNodeParams& params) : 
RosServiceNode<ant_fleet_interfaces::srv::MoveOut>(name, conf, params) {}

PortsList MoveOut::providedPorts()
{
  return {};
}

bool MoveOut::setRequest(Request::SharedPtr& request)
{
  return true;
}

NodeStatus MoveOut::onResponseReceived(const Response::SharedPtr& response)
{
  if (auto node = node_.lock())  // Attempt to lock the weak_ptr
  {
      RCLCPP_INFO(node->get_logger(), "[%s] MoveOut complete ? %d", this->name().c_str(), response->success);
  }

  if(response -> success)
  {
    return NodeStatus::SUCCESS;
  }

  return NodeStatus::FAILURE;
}

NodeStatus MoveOut::onFailure(ServiceNodeErrorCode error)
{
  return NodeStatus::FAILURE;
}
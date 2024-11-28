#include "nodes.h"


CheckIdle::CheckIdle(const std::string& name, const NodeConfig& conf, const RosNodeParams& params) : RosServiceNode<CheckDroneIdle_srv_t>(name, conf, params) {}

PortsList CheckIdle::providedPorts()
{
  return {};
}

bool CheckIdle::setRequest(Request::SharedPtr& request)
{
  return true;
}

NodeStatus CheckIdle::onResponseReceived(const Response::SharedPtr& response)
{
  if (auto node = node_.lock())  // Attempt to lock the weak_ptr
  {
      RCLCPP_INFO(node->get_logger(), "[%s] Is drone idle?: %d", this->name().c_str(), response->is_drone_idle);
  }

  if(response -> is_drone_idle)
  {
    return NodeStatus::SUCCESS;
  }

  return NodeStatus::FAILURE;
}

NodeStatus CheckIdle::onFailure(ServiceNodeErrorCode error)
{
  return NodeStatus::FAILURE;
}
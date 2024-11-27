#include "nodes.h"


CheckIdle::CheckIdle(const std::string& name, const NodeConfig& conf, const RosNodeParams& params) : RosServiceNode<CheckDroneIdle_srv_t>(name, conf, params) {}

PortsList CheckIdle::providedPorts()
{
  return {};
  // providedBasicPorts({
  //     InputPort<unsigned>("A"),
  //     InputPort<unsigned>("B")});
}

bool CheckIdle::setRequest(Request::SharedPtr& request)
{
    
  // use input ports to set A and B
  // getInput("A", request->a);
  // getInput("B", request->b);
  // must return true if we are ready to send the request
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
  // RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
  return NodeStatus::FAILURE;
}
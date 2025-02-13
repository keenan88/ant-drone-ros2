#include "nodes.h"


SuspendReleaseRMFPathing::SuspendReleaseRMFPathing(const std::string& name, const NodeConfig& conf, const RosNodeParams& params) : RosServiceNode<antdrone_interfaces::srv::SuspendReleaseRMFPathing>(name, conf, params) {}

PortsList SuspendReleaseRMFPathing::providedPorts()
{
  return {InputPort<std::string>("set_state")};
}

bool SuspendReleaseRMFPathing::setRequest(Request::SharedPtr& request)
{
  bool request_ready = false;

  std::string set_state;
  getInput("set_state", set_state);

  if(set_state == "suspend")
  {
    request -> is_rmf_pathing_suspended = true;
    request_ready = true;
  }
  else if (set_state == "release")
  {
    request -> is_rmf_pathing_suspended = false;
    request_ready = true;
  }
  
  return request_ready;
}

NodeStatus SuspendReleaseRMFPathing::onResponseReceived(const Response::SharedPtr& response)
{
  return NodeStatus::SUCCESS;
}

NodeStatus SuspendReleaseRMFPathing::onFailure(ServiceNodeErrorCode error)
{
  if(error){
    if (auto node = node_.lock()) {
      RCLCPP_INFO(node->get_logger(), "[%s] Error calling service", this->name().c_str());
    }
  }
  return NodeStatus::FAILURE;
}
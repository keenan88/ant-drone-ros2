#include "nodes.h"


SuspendRMFPathing::SuspendRMFPathing(const std::string& name, const NodeConfig& conf, const RosNodeParams& params) : RosServiceNode<antdrone_interfaces::srv::SuspendRMFPathing>(name, conf, params) {}

PortsList SuspendRMFPathing::providedPorts()
{
  return {};
}

bool SuspendRMFPathing::setRequest(Request::SharedPtr& request)
{
  request -> is_rmf_pathing_suspended = true;
    
  return true;
}

NodeStatus SuspendRMFPathing::onResponseReceived(const Response::SharedPtr& response)
{
  if(response -> success)
  {
    return NodeStatus::SUCCESS;
  }

  return NodeStatus::FAILURE;
}

NodeStatus SuspendRMFPathing::onFailure(ServiceNodeErrorCode error)
{
  if(error){} // To avoid build warning
  return NodeStatus::FAILURE;
}
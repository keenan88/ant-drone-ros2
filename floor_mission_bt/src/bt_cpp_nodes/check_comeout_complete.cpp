#include "nodes.h"


CheckComeOutComplete::CheckComeOutComplete(const std::string& name, const NodeConfig& conf, const RosNodeParams& params) : RosServiceNode<ant_fleet_interfaces::srv::CheckIfComeOutTriggered>(name, conf, params) {}

PortsList CheckComeOutComplete::providedPorts()
{
  return {InputPort<std::string>("worker_name")};
}

bool CheckComeOutComplete::setRequest(Request::SharedPtr& request)
{
  getInput("worker_name", request -> worker_name);
    
  return true;
}

NodeStatus CheckComeOutComplete::onResponseReceived(const Response::SharedPtr& response)
{
  if(response -> is_triggered)
  {
    return NodeStatus::SUCCESS;
  }

  return NodeStatus::FAILURE;
}

NodeStatus CheckComeOutComplete::onFailure(ServiceNodeErrorCode error)
{
  return NodeStatus::FAILURE;
}
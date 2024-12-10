#include "nodes.h"


SendComeOutTrigger::SendComeOutTrigger(const std::string& name, const NodeConfig& conf, const RosNodeParams& params) : RosServiceNode<ant_fleet_interfaces::srv::CheckIfComeOutTriggered>(name, conf, params) {}

PortsList SendComeOutTrigger::providedPorts()
{
  return {InputPort<std::string>("worker_name")};
}

bool SendComeOutTrigger::setRequest(Request::SharedPtr& request)
{
  getInput("worker_name", request -> worker_name);
  request -> is_posting = true;
  request -> is_triggered = true;
    
  return true;
}

NodeStatus SendComeOutTrigger::onResponseReceived(const Response::SharedPtr& response)
{
   return NodeStatus::SUCCESS;
}

NodeStatus SendComeOutTrigger::onFailure(ServiceNodeErrorCode error)
{
  return NodeStatus::FAILURE;
}
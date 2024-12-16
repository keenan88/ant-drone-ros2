#include "nodes.h"


SendComeOutTrigger::SendComeOutTrigger(const std::string& name, const NodeConfig& conf, const RosNodeParams& params) : RosServiceNode<ant_queen_interfaces::srv::ComeOut>(name, conf, params) {}

PortsList SendComeOutTrigger::providedPorts()
{
  return {InputPort<std::string>("worker_name")};
}

bool SendComeOutTrigger::setRequest(Request::SharedPtr& request)
{
  getInput("worker_name", request -> worker_name);
  request -> set_trigger = true;
    
  return true;
}

NodeStatus SendComeOutTrigger::onResponseReceived(const Response::SharedPtr& response)
{
  if(response) {} // To avoid build warning
  return NodeStatus::SUCCESS;
}

NodeStatus SendComeOutTrigger::onFailure(ServiceNodeErrorCode error)
{
  if(error){} // To avoid build warning
  return NodeStatus::FAILURE;
}
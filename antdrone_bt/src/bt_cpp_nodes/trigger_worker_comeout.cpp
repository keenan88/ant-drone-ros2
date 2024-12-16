#include "nodes.h"


TriggerWorkerComeout::TriggerWorkerComeout(const std::string& name, const NodeConfig& conf, const RosNodeParams& params) : RosServiceNode<ant_queen_interfaces::srv::ComeOut>(name, conf, params) {}

PortsList TriggerWorkerComeout::providedPorts()
{
  return {InputPort<std::string>("worker_name")};
}

bool TriggerWorkerComeout::setRequest(Request::SharedPtr& request)
{
  getInput("worker_name", request -> worker_name);
  request -> set_trigger = true;
    
  return true;
}

NodeStatus TriggerWorkerComeout::onResponseReceived(const Response::SharedPtr& response)
{
  if(response) {} // To avoid build warning
  return NodeStatus::SUCCESS;
}

NodeStatus TriggerWorkerComeout::onFailure(ServiceNodeErrorCode error)
{
  if(error){} // To avoid build warning
  return NodeStatus::FAILURE;
}
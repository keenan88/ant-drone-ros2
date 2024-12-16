#include "nodes.h"


ClearCostmap::ClearCostmap(const std::string& name, const NodeConfig& conf, const RosNodeParams& params) : RosServiceNode<nav2_msgs::srv::ClearEntireCostmap>(name, conf, params) {}

PortsList ClearCostmap::providedPorts()
{
  return {};
}

bool ClearCostmap::setRequest(Request::SharedPtr& request)
{   
  if(request){} // To avoid build warning
  return true;
}

NodeStatus ClearCostmap::onResponseReceived(const Response::SharedPtr& response)
{
   if(response) {} // To avoid build warning
   return NodeStatus::SUCCESS;
}

NodeStatus ClearCostmap::onFailure(ServiceNodeErrorCode error)
{
  if(error){} // To avoid build warning
  return NodeStatus::FAILURE;
}
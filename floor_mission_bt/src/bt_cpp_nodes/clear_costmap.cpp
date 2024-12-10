#include "nodes.h"


ClearCostmap::ClearCostmap(const std::string& name, const NodeConfig& conf, const RosNodeParams& params) : RosServiceNode<nav2_msgs::srv::ClearEntireCostmap>(name, conf, params) {}

PortsList ClearCostmap::providedPorts()
{
  return {};
}

bool ClearCostmap::setRequest(Request::SharedPtr& request)
{   
  return true;
}

NodeStatus ClearCostmap::onResponseReceived(const Response::SharedPtr& response)
{
   return NodeStatus::SUCCESS;
}

NodeStatus ClearCostmap::onFailure(ServiceNodeErrorCode error)
{
  return NodeStatus::FAILURE;
}
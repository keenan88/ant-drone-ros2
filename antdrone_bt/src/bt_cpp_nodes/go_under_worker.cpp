#include "nodes.h"

GoUnderWorker::GoUnderWorker(const std::string &name, const NodeConfig &conf, const RosNodeParams &params) : RosServiceNode<antdrone_interfaces::srv::GoUnder>(name, conf, params) {}

PortsList GoUnderWorker::providedPorts() { return {}; }

bool GoUnderWorker::setRequest(Request::SharedPtr &request) {
  
  return true;
}

NodeStatus GoUnderWorker::onResponseReceived(const Response::SharedPtr &response) {
  if (response->success) {
    return NodeStatus::SUCCESS;
  } else {
    return NodeStatus::FAILURE;
  }

  return NodeStatus::RUNNING;
}

NodeStatus GoUnderWorker::onFailure(ServiceNodeErrorCode error) {
  if (error) {
  } // To avoid build warning
  return NodeStatus::FAILURE;
}
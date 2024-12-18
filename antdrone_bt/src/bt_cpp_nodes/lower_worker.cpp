#include "nodes.h"

LowerWorker::LowerWorker(const std::string &name, const NodeConfig &conf, const RosNodeParams &params) : RosServiceNode<linkattacher_msgs::srv::DetachLink>(name, conf, params) {}

PortsList LowerWorker::providedPorts() { return providedBasicPorts({InputPort<std::string>("drone_name"), InputPort<std::string>("worker_name")}); }

bool LowerWorker::setRequest(Request::SharedPtr &request) {
  getInput("drone_name", request->model1_name);
  getInput("worker_name", request->model2_name);
  request->link1_name = "attachment_point";
  request->link2_name = "base_link";

  return true;
}

NodeStatus LowerWorker::onResponseReceived(const Response::SharedPtr &response) {
  if (response->success) {
    return NodeStatus::SUCCESS;
  } else {
    return NodeStatus::FAILURE;
  }

  return NodeStatus::RUNNING;
}

NodeStatus LowerWorker::onFailure(ServiceNodeErrorCode error) {
  if (error) {
  } // To avoid build warning
  return NodeStatus::FAILURE;
}
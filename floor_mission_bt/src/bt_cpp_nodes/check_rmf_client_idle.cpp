#include "nodes.h"

CheckRMFClientIdle::CheckRMFClientIdle(const std::string &name, const NodeConfig &conf,
                     const RosNodeParams &params)
    : RosServiceNode<antdrone_interfaces::srv::CheckRMFClientIdle>(name, conf,
                                                               params) {}

PortsList CheckRMFClientIdle::providedPorts() { return {}; }

bool CheckRMFClientIdle::setRequest(Request::SharedPtr &request) { 
  if(request){} // To avoid build warning
  return true; 
}

NodeStatus CheckRMFClientIdle::onResponseReceived(const Response::SharedPtr &response) {
  if (auto node = node_.lock()) {
    RCLCPP_INFO(node->get_logger(), "[%s] Is drone idle?: %d",
                this->name().c_str(), response->is_rmf_client_idle);
  }

  if (response->is_rmf_client_idle) {
    return NodeStatus::SUCCESS;
  }

  return NodeStatus::FAILURE;
}

NodeStatus CheckRMFClientIdle::onFailure(ServiceNodeErrorCode error) {
  if(error){} // To avoid build warning
  return NodeStatus::FAILURE;
}
#include "nodes.h"


SendSuspendRMFPathing::SendSuspendRMFPathing(const std::string& name, const NodeConfig& conf, const RosNodeParams& params) : RosServiceNode<SuspendRMFPathing_srv_t>(name, conf, params) {}

PortsList SendSuspendRMFPathing::providedPorts()
{
  return {};
  // providedBasicPorts({
  //     InputPort<unsigned>("A"),
  //     InputPort<unsigned>("B")});
}

bool SendSuspendRMFPathing::setRequest(Request::SharedPtr& request)
{
  request -> is_rmf_pathing_suspended = true;
    
  // use input ports to set A and B
  // getInput("A", request->a);
  // getInput("B", request->b);
  // must return true if we are ready to send the request
  return true;
}

NodeStatus SendSuspendRMFPathing::onResponseReceived(const Response::SharedPtr& response)
{
  // RCLCPP_INFO(node_->get_logger(), "Sum: %ld", response->sum);
  if(response -> success)
  {
    return NodeStatus::SUCCESS;
  }

  return NodeStatus::FAILURE;
}

NodeStatus SendSuspendRMFPathing::onFailure(ServiceNodeErrorCode error)
{
  // RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
  return NodeStatus::FAILURE;
}
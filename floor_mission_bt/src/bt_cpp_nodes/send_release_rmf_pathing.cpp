#include "nodes.h"


SendReleaseRMFPathing::SendReleaseRMFPathing(const std::string& name, const NodeConfig& conf, const RosNodeParams& params) : RosServiceNode<SuspendRMFPathing_srv_t>(name, conf, params) {}

PortsList SendReleaseRMFPathing::providedPorts()
{
  return {};
  // providedBasicPorts({
  //     InputPort<unsigned>("A"),
  //     InputPort<unsigned>("B")});
}

bool SendReleaseRMFPathing::setRequest(Request::SharedPtr& request)
{
  request -> is_rmf_pathing_suspended = false;
    
  // use input ports to set A and B
  // getInput("A", request->a);
  // getInput("B", request->b);
  // must return true if we are ready to send the request
  return true;
}

NodeStatus SendReleaseRMFPathing::onResponseReceived(const Response::SharedPtr& response)
{
  // RCLCPP_INFO(node_->get_logger(), "Sum: %ld", response->sum);
  if(response -> success)
  {
    return NodeStatus::SUCCESS;
  }

  return NodeStatus::FAILURE;
}

NodeStatus SendReleaseRMFPathing::onFailure(ServiceNodeErrorCode error)
{
  // RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
  return NodeStatus::FAILURE;
}
#include "nodes.h"


ReleaseRMFPathing::ReleaseRMFPathing(const std::string& name, const NodeConfig& conf, const RosNodeParams& params) : RosServiceNode<antdrone_interfaces::srv::SuspendRMFPathing>(name, conf, params) {}

PortsList ReleaseRMFPathing::providedPorts()
{
  return {};
  // providedBasicPorts({
  //     InputPort<unsigned>("A"),
  //     InputPort<unsigned>("B")});
}

bool ReleaseRMFPathing::setRequest(Request::SharedPtr& request)
{
  request -> is_rmf_pathing_suspended = false;
    
  // use input ports to set A and B
  // getInput("A", request->a);
  // getInput("B", request->b);
  // must return true if we are ready to send the request
  return true;
}

NodeStatus ReleaseRMFPathing::onResponseReceived(const Response::SharedPtr& response)
{
  // RCLCPP_INFO(node_->get_logger(), "Sum: %ld", response->sum);
  if(response -> success)
  {
    return NodeStatus::SUCCESS;
  }

  return NodeStatus::FAILURE;
}

NodeStatus ReleaseRMFPathing::onFailure(ServiceNodeErrorCode error)
{
  if(error){} // To avoid build warning
  // RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
  return NodeStatus::FAILURE;
}
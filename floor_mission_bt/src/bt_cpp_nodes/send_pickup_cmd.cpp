#include "nodes.h"


SendPickupCmd::SendPickupCmd(const std::string& name, const NodeConfig& conf, const RosNodeParams& params) : RosServiceNode<SendPickupCmd_srv_t>(name, conf, params) {}

PortsList SendPickupCmd::providedPorts()
{
  return {};
  // providedBasicPorts({
  //     InputPort<unsigned>("A"),
  //     InputPort<unsigned>("B")});
}

bool SendPickupCmd::setRequest(Request::SharedPtr& request)
{
  request -> model1_name = "drone_boris";
  request -> model2_name = "worker_misha";
  request -> link1_name = request -> model1_name + "_attachment_point";
  request -> link2_name = request -> model2_name + "_base_link";    
    
  // use input ports to set A and B
  // getInput("A", request->a);
  // getInput("B", request->b);
  // must return true if we are ready to send the request
  return true;
}

NodeStatus SendPickupCmd::onResponseReceived(const Response::SharedPtr& response)
{
  // RCLCPP_INFO(node_->get_logger(), "Sum: %ld", response->sum);
  if(response -> success)
  {
    return NodeStatus::SUCCESS;
  }

  return NodeStatus::FAILURE;
}

NodeStatus SendPickupCmd::onFailure(ServiceNodeErrorCode error)
{
  // RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
  return NodeStatus::FAILURE;
}
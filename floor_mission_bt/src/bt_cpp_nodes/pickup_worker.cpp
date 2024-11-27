#include "nodes.h"


PickupWorker::PickupWorker(const std::string& name, const NodeConfig& conf, const RosNodeParams& params) : RosServiceNode<SendPickupCmd_srv_t>(name, conf, params) {}

PortsList PickupWorker::providedPorts()
{
  return {};
  // providedBasicPorts({
  //     InputPort<unsigned>("A"),
  //     InputPort<unsigned>("B")});
}

bool PickupWorker::setRequest(Request::SharedPtr& request)
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

NodeStatus PickupWorker::onResponseReceived(const Response::SharedPtr& response)
{
  if(response -> success)
  {
    return NodeStatus::SUCCESS;
  }
  else
  {
    return NodeStatus::FAILURE;
  }

  return NodeStatus::RUNNING;
}

NodeStatus PickupWorker::onFailure(ServiceNodeErrorCode error)
{
  // RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
  return NodeStatus::FAILURE;
}
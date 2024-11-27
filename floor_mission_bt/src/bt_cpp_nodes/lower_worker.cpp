#include "nodes.h"


LowerWorker::LowerWorker(const std::string& name, const NodeConfig& conf, const RosNodeParams& params) : RosServiceNode<SendLowerCmd_srv_t>(name, conf, params) {}

PortsList LowerWorker::providedPorts()
{
  return {};
  // providedBasicPorts({
  //     InputPort<unsigned>("A"),
  //     InputPort<unsigned>("B")});
}

bool LowerWorker::setRequest(Request::SharedPtr& request)
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

NodeStatus LowerWorker::onResponseReceived(const Response::SharedPtr& response)
{
  // RCLCPP_INFO(node_->get_logger(), "Sum: %ld", response->sum);
  if(response -> success)
  {
    return NodeStatus::SUCCESS;
  }

  return NodeStatus::FAILURE;
}

NodeStatus LowerWorker::onFailure(ServiceNodeErrorCode error)
{
  // RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
  return NodeStatus::FAILURE;
}
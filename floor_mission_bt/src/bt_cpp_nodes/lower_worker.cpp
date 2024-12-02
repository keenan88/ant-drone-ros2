#include "nodes.h"


LowerWorker::LowerWorker(const std::string& name, const NodeConfig& conf, const RosNodeParams& params) : RosServiceNode<SendLowerCmd_srv_t>(name, conf, params) {}

PortsList LowerWorker::providedPorts()
{
  return providedBasicPorts({
      InputPort<std::string>("drone_name"),
      InputPort<std::string>("worker_name")
  });
}

bool LowerWorker::setRequest(Request::SharedPtr& request)
{
  getInput("drone_name", request -> model1_name);
  getInput("worker_name", request -> model2_name);
  request -> link1_name = request -> model1_name + "_attachment_point";
  request -> link2_name = request -> model2_name + "_base_link";    
    
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
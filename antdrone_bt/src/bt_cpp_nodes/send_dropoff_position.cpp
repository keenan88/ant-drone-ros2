#include "nodes.h"
#include <cmath> // For std::atan2

SendDropoffPosition::SendDropoffPosition(const std::string &name, const NodeConfig &conf, const RosNodeParams &params) : RosServiceNode<ant_queen_interfaces::srv::DropoffPos>(name, conf, params) {
  if (auto node = node_.lock()) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());

    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }
}

PortsList SendDropoffPosition::providedPorts() { return {InputPort<std::string>("drone_name"), InputPort<std::string>("pickup_side")}; }

geometry_msgs::msg::TransformStamped SendDropoffPosition::getTransform() {
  std::string pickup_side;
  getInput("pickup_side", pickup_side);

  const std::string target_frame = "map";
  const std::string source_frame = "attachment_point_" + pickup_side;

  geometry_msgs::msg::TransformStamped transform_stamped;

  try {
    transform_stamped = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);

  } catch (tf2::TransformException &ex) {
    if (auto node = node_.lock()) {
      RCLCPP_WARN(node->get_logger(), "Could not transform %s to %s: %s", source_frame.c_str(), target_frame.c_str(), ex.what());
    }
    transform_stamped.transform.rotation.w = 2.0; // Set w to impossible value to flag error
  }

  return transform_stamped;
}

bool SendDropoffPosition::setRequest(Request::SharedPtr &request) {

  std::string drone_name;

  getInput("drone_name", drone_name);

  geometry_msgs::msg::TransformStamped map_to_attachment_point_tf = getTransform();

  if (map_to_attachment_point_tf.transform.rotation.w <= 1.0) {
    request->drone_name = drone_name;
    request->x = map_to_attachment_point_tf.transform.translation.x;
    request->y = map_to_attachment_point_tf.transform.translation.y;

    float w = map_to_attachment_point_tf.transform.rotation.w;
    float x = map_to_attachment_point_tf.transform.rotation.x;
    float y = map_to_attachment_point_tf.transform.rotation.y;
    float z = map_to_attachment_point_tf.transform.rotation.z;
    float yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

    request->yaw = yaw;

    if (auto node = node_.lock()) {
      RCLCPP_INFO(node->get_logger(), "[%s] Sending dropoff position..", this->name().c_str());
    }
  } else {
    return false;
  }

  // must return true if we are ready to send the request
  return true;
}

NodeStatus SendDropoffPosition::onResponseReceived(const Response::SharedPtr &response) {
  if (response) {
    return NodeStatus::SUCCESS;
  }

  return NodeStatus::FAILURE;
}

NodeStatus SendDropoffPosition::onFailure(ServiceNodeErrorCode error) {
  if (error) {
  } // To avoid build warning
  // RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
  return NodeStatus::FAILURE;
}
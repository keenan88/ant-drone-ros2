#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <nav2_msgs/srv/set_initial_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_msgs/msg/tf_message.hpp>
#include <vector>

class GZFrameNameFixer : public rclcpp::Node {
public:
  GZFrameNameFixer() : Node("tf_filter_republisher") {
    drone_name_ = this->declare_parameter<std::string>("DRONE_NAME", "");

    tf_subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>("/tf_gz", 10, std::bind(&GZFrameNameFixer::tfCallback, this, std::placeholders::_1));

    tf_publisher_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
  }

private:
  void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    tf2_msgs::msg::TFMessage filtered_msg;

    for (const auto &transform : msg->transforms) {
      if (transform.header.frame_id.find(drone_name_) != std::string::npos || transform.child_frame_id.find(drone_name_) != std::string::npos) {
        geometry_msgs::msg::TransformStamped new_transform = transform;

        new_transform.header.frame_id.erase(0, drone_name_.length() + 1);
        new_transform.child_frame_id.erase(0, drone_name_.length() + 1);

        filtered_msg.transforms.push_back(new_transform);
      }
    }

    if (!filtered_msg.transforms.empty()) {
      tf_publisher_->publish(filtered_msg);
    }
  }

  std::string drone_name_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscription_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_subscription_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_publisher_;
  rclcpp::Client<nav2_msgs::srv::SetInitialPose>::SharedPtr set_initial_pose_client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GZFrameNameFixer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

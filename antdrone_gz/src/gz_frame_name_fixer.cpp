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

    // Initialize starting pose
    auto starting_pose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    starting_pose->header.stamp = this->get_clock()->now();
    starting_pose->header.frame_id = "map";
    starting_pose->pose.pose.orientation.x = 0.0;
    starting_pose->pose.pose.orientation.y = 0.0;
    starting_pose->pose.pose.orientation.z = 0.0;
    starting_pose->pose.pose.orientation.w = 1.0;
    starting_pose->pose.pose.position.z = 0.0;

    if (drone_name_ == "drone_boris") {
      starting_pose->pose.pose.position.x = 16.5;
      starting_pose->pose.pose.position.y = -18.6;

      set_initial_pose_client_ = this->create_client<nav2_msgs::srv::SetInitialPose>("set_initial_pose");
      uint8_t tries = 0;
      while (!set_initial_pose_client_->wait_for_service(std::chrono::seconds(2)) && tries <= 5) {
        RCLCPP_INFO(this->get_logger(), "set_initial_pose service not available, waiting again...");
        tries++;
      }
      if (set_initial_pose_client_->wait_for_service(std::chrono::seconds(1))) {
        auto request = std::make_shared<nav2_msgs::srv::SetInitialPose::Request>();
        request->pose = *starting_pose;

        auto future = set_initial_pose_client_->async_send_request(request);
        rclcpp::spin_until_future_complete(this->shared_from_this(), future);
      } else {
        RCLCPP_INFO(this->get_logger(), "set_initial_pose service not available, moving on");
      }
    }
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

  std::string replaceSubstring(const std::string &str, const std::string &from, const std::string &to) {

    std::string result = str;
    size_t start_pos = result.find(from);
    if (start_pos != std::string::npos) {
      result.replace(start_pos, from.length(), to);
    }
    return result;
  }

  std::string drone_name_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscription_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_;
  rclcpp::Client<nav2_msgs::srv::SetInitialPose>::SharedPtr set_initial_pose_client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GZFrameNameFixer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

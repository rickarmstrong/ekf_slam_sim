// Copyright (c) 2025 Richard Armstrong
# include "ekf_localizer.hpp"

class EkfNode final : public rclcpp::Node
{
public:
  EkfNode()
    : Node("ekf_localizer")
  {
    Ekf filter_ = Ekf();
    tag_detections_sub_ = this->create_subscription<TagArray>(
            "tag_detections", 10, [this](const TagArray::SharedPtr msg){ detection_cb(msg); });
  }

private:
  void detection_cb(const TagArray::SharedPtr& msg) const {
    std::cout << msg->header.frame_id << std::endl;
  }
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<TagArray>::SharedPtr tag_detections_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<EkfNode>();
  RCLCPP_INFO(node->get_logger(), "EKF Localizer node starting.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

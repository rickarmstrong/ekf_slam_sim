#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "apriltag_ros_interfaces/msg/april_tag_detection.hpp"
#include "apriltag_ros_interfaces/msg/april_tag_detection_array.hpp"

using std::placeholders::_1;

class EKFLocalizer : public rclcpp::Node
{
public:
    EKFLocalizer()
        : Node("ekf_localizer")
    {
        tag_detections_sub_ = this->create_subscription<apriltag_ros_interfaces::msg::AprilTagDetectionArray>(
                "tag_detections", 10, std::bind(&EKFLocalizer::detection_cb, this, _1));
    }

private:
    void detection_cb(const apriltag_ros_interfaces::msg::AprilTagDetectionArray::SharedPtr msg) const {}
    rclcpp::Subscription<apriltag_ros_interfaces::msg::AprilTagDetectionArray>::SharedPtr tag_detections_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    const auto node = std::make_shared<EKFLocalizer>();
    RCLCPP_INFO(node->get_logger(), "EKF Localizer node starting.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
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
        subscription_ = this->create_subscription<std_msgs::msg::String>(
                "topic", 10, std::bind(&EKFLocalizer::topic_callback, this, _1));
        tag_detections_sub_ = this->create_subscription<apriltag_ros_interfaces::msg::AprilTagDetectionArray>(
                "/tag_detections", 10, std::bind(&EKFLocalizer::detection_cb, this, _1));
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    void detection_cb(const apriltag_ros_interfaces::msg::AprilTagDetectionArray::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Tag detected.");
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Subscription<apriltag_ros_interfaces::msg::AprilTagDetectionArray>::SharedPtr tag_detections_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EKFLocalizer>());
    rclcpp::shutdown();
    return 0;
}
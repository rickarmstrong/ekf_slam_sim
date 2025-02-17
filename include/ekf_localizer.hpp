//
// Created by RDA on 2/17/25.
//
#ifndef EKF_LOCALIZER_HPP
#define EKF_LOCALIZER_HPP
#include <memory>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "apriltag_ros_interfaces/msg/april_tag_detection.hpp"
#include "apriltag_ros_interfaces/msg/april_tag_detection_array.hpp"

// Aliases to reduce the name noise.
using TagDetection = apriltag_ros_interfaces::msg::AprilTagDetection;
using  TagArray = apriltag_ros_interfaces::msg::AprilTagDetectionArray;

// The number of tags defined in config/tags.yaml.
constexpr int LANDMARKS_KNOWN = 16;

class EKFLocalizer : public rclcpp::Node
{
public:
    EKFLocalizer()
        : Node("ekf_localizer")
    {
        tag_detections_sub_ = this->create_subscription<TagArray>(
                "tag_detections", 10, [this](const TagArray::SharedPtr msg){ detection_cb(msg); });
    }

private:
    void detection_cb(const TagArray::SharedPtr& msg) const {
        std::cout << msg->header.frame_id << std::endl;
    }
    rclcpp::Subscription<TagArray>::SharedPtr tag_detections_sub_;
};
#endif //EKF_LOCALIZER_HPP

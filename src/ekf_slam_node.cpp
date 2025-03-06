// Copyright (c) 2025 Richard Armstrong
# include "ekf_localizer.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    const auto node = std::make_shared<EKFLocalizerNode>();
    RCLCPP_INFO(node->get_logger(), "EKF Localizer node starting.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
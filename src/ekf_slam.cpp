# include "ekf_localizer.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    const auto node = std::make_shared<EKFLocalizer>();
    RCLCPP_INFO(node->get_logger(), "EKF Localizer node starting.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
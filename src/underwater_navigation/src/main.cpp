#include "underwater_navigation/imu_subscriber.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ImuSubscriber>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}

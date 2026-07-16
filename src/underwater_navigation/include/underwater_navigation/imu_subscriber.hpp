#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <deque>
#include <string>

#include "underwater_navigation/GraphManager.hpp"

class ImuSubscriber : public rclcpp::Node
{
public:
    ImuSubscriber();

private:
    void callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void depthCallback(const sensor_msgs::msg::FluidPressure::SharedPtr msg);
    void dvlCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void processBuffers();

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr depth_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr dvl_sub_;
    std::shared_ptr<GraphManager> graph_manager_;
    std::deque<GraphManager::TimedImuMeasurement> imu_buffer_;
    std::deque<GraphManager::TimedDepthMeasurement> depth_buffer_;
    std::deque<GraphManager::TimedDvlMeasurement> dvl_buffer_;
    rclcpp::Time last_processed_imu_stamp_;
    size_t imu_message_count_;
    int imu_window_size_;
    std::string imu_topic_;
    std::string depth_topic_;
    std::string dvl_topic_;
    std::string mag_topic_;
};

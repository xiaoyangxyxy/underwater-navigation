#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/range.hpp>

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
    void magCallback(const sensor_msgs::msg::MagneticField::SharedPtr msg);
    void sonarUpCallback(const sensor_msgs::msg::Range::SharedPtr msg);
    void sonarDownCallback(const sensor_msgs::msg::Range::SharedPtr msg);
    void sonarLeftCallback(const sensor_msgs::msg::Range::SharedPtr msg);
    void sonarRightCallback(const sensor_msgs::msg::Range::SharedPtr msg);
    void sonarCallback(
        const sensor_msgs::msg::Range::SharedPtr msg,
        const std::string& name,
        const gtsam::Vector3& offset_body,
        const gtsam::Vector3& direction_body);
    void processBuffers();

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr depth_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr dvl_sub_;
    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sonar_up_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sonar_down_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sonar_left_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sonar_right_sub_;
    std::shared_ptr<GraphManager> graph_manager_;
    std::deque<GraphManager::TimedImuMeasurement> imu_buffer_;
    std::deque<GraphManager::TimedDepthMeasurement> depth_buffer_;
    std::deque<GraphManager::TimedDvlMeasurement> dvl_buffer_;
    std::deque<GraphManager::TimedMagMeasurement> mag_buffer_;
    std::deque<GraphManager::TimedSonarMeasurement> sonar_buffer_;
    rclcpp::Time last_processed_imu_stamp_;
    size_t imu_message_count_;
    int imu_window_size_;
    std::string imu_topic_;
    std::string depth_topic_;
    std::string dvl_topic_;
    std::string mag_topic_;
    std::string sonar_up_topic_;
    std::string sonar_down_topic_;
    std::string sonar_left_topic_;
    std::string sonar_right_topic_;
    gtsam::Vector3 sonar_up_offset_body_;
    gtsam::Vector3 sonar_down_offset_body_;
    gtsam::Vector3 sonar_left_offset_body_;
    gtsam::Vector3 sonar_right_offset_body_;
};

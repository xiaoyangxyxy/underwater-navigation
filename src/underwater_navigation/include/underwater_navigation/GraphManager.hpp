#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/NavState.h>

#include <deque>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>

#include "underwater_navigation/imu/ImuFactorManager.hpp"

class GraphManager
{
public:
    struct TimedImuMeasurement
    {
        rclcpp::Time stamp;
        gtsam::Vector3 acc;
        gtsam::Vector3 gyro;
    };

    struct TimedDepthMeasurement
    {
        rclcpp::Time stamp;
        double depth;
    };

    struct TimedDvlMeasurement
    {
        rclcpp::Time stamp;
        gtsam::Vector3 velocity_body;
    };

    struct ImuWindowKeys
    {
        gtsam::Key pose_j;
        gtsam::Key vel_j;
        gtsam::Key bias_j;
        gtsam::Key z_j;
    };

    explicit GraphManager(rclcpp::Node* node = nullptr);

    void addPriorPose();
    void addBetweenPose();
    void addPriorVelocity();
    void addPriorBias();
    void optimize();

    void addImuMeasurement(
        const gtsam::Vector3& acc,
        const gtsam::Vector3& gyro,
        double dt
    );
    void addDepthMeasurement(
        gtsam::Key pose_key,
        gtsam::Key z_key,
        double depth
    );

    bool processBuffers(
        std::deque<TimedImuMeasurement>& imu_buffer,
        std::deque<TimedDepthMeasurement>& depth_buffer,
        std::deque<TimedDvlMeasurement>& dvl_buffer,
        rclcpp::Time& last_processed_imu_stamp
    );

    void addDvlMeasurement(
        const gtsam::Key& pose_key,
        const gtsam::Key& velocity_key,
        const gtsam::Vector3& velocity_body
    );
    void addDvlDisplacementMeasurement(
        const gtsam::Key& pose_i,
        const gtsam::Key& pose_j,
        const gtsam::Vector3& velocity_body,
        double dt
    );
    ImuWindowKeys addImuFactor();
    void printImuPreintegration() const;
    void publishPath(const rclcpp::Time& stamp);

    const gtsam::imuBias::ConstantBias& currentBias() const;

private:
    void groundTruthCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void publishDebugLog(const rclcpp::Time& stamp);

    gtsam::NonlinearFactorGraph graph_;

    gtsam::Values new_values_;

    gtsam::ISAM2 isam2_;

    gtsam::Key current_pose_key_;
    gtsam::Key current_velocity_key_;
    gtsam::Vector3 current_velocity_estimate_;
    gtsam::Key current_bias_key_;
    gtsam::imuBias::ConstantBias current_bias_estimate_;
    gtsam::Key current_z_key_;
    double current_z_estimate_;
    double last_depth_measurement_;
    double depth_sigma_;
    double pose_z_sigma_;
    double initial_velocity_prior_sigma_;
    double dvl_match_max_dt_;
    bool dvl_displacement_factor_enabled_;
    double dvl_displacement_sigma_;
    bool debug_attitude_prior_enabled_;
    double debug_attitude_prior_sigma_;
    bool has_depth_measurement_;
    bool has_ground_truth_;
    gtsam::Point3 latest_ground_truth_position_;
    rclcpp::Time latest_ground_truth_stamp_;

    bool latest_dvl_matched_;
    double latest_dvl_time_diff_;
    gtsam::Vector3 latest_dvl_measured_;
    gtsam::Key latest_dvl_pose_key_;
    gtsam::Key latest_dvl_velocity_key_;
    rclcpp::Time latest_debug_stamp_;

    size_t frame_index_;

    gtsam::Pose3 current_pose_estimate_;
    gtsam::NavState current_nav_state_;

    ImuFactorManager imu_manager_;

    rclcpp::Logger logger_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ground_truth_sub_;
    nav_msgs::msg::Path path_msg_;
};

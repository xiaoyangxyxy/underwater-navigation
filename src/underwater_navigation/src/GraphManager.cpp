#include "underwater_navigation/GraphManager.hpp"

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/Vector.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>

namespace
{
class PoseZFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3, double>
{
public:
    PoseZFactor(
        gtsam::Key pose_key,
        gtsam::Key z_key,
        const gtsam::SharedNoiseModel& noise_model)
    : gtsam::NoiseModelFactorN<gtsam::Pose3, double>(
          noise_model,
          pose_key,
          z_key)
    {
    }

    gtsam::Vector evaluateError(
        const gtsam::Pose3& pose,
        const double& z,
        gtsam::OptionalMatrixType H_pose,
        gtsam::OptionalMatrixType H_z) const override
    {
        if (H_pose)
        {
            *H_pose = gtsam::Matrix::Zero(1, 6);
            const auto translation_interval =
                gtsam::Pose3::translationInterval();
            H_pose->block<1, 3>(0, translation_interval.first) =
                pose.rotation().matrix().row(2);
        }

        if (H_z)
        {
            *H_z = gtsam::Matrix::Constant(1, 1, -1.0);
        }

        return (gtsam::Vector(1) << pose.translation().z() - z).finished();
    }
};

class DvlVelocityFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3>
{
public:
    DvlVelocityFactor(
        gtsam::Key pose_key,
        gtsam::Key velocity_key,
        const gtsam::Vector3& measured_velocity_body,
        const gtsam::SharedNoiseModel& noise_model)
    : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3>(
          noise_model,
          pose_key,
          velocity_key),
      measured_velocity_body_(measured_velocity_body)
    {
    }

    gtsam::Vector evaluateError(
        const gtsam::Pose3& pose,
        const gtsam::Vector3& velocity_world,
        gtsam::OptionalMatrixType H_pose,
        gtsam::OptionalMatrixType H_velocity) const override
    {
        const auto error_function =
            [this](const gtsam::Pose3& p, const gtsam::Vector3& v) {
                return predictVelocityBody(p, v) - measured_velocity_body_;
            };

        if (H_pose)
        {
            *H_pose = gtsam::numericalDerivative21<gtsam::Vector3, gtsam::Pose3, gtsam::Vector3>(
                error_function,
                pose,
                velocity_world);
        }

        if (H_velocity)
        {
            *H_velocity = gtsam::numericalDerivative22<gtsam::Vector3, gtsam::Pose3, gtsam::Vector3>(
                error_function,
                pose,
                velocity_world);
        }

        return error_function(pose, velocity_world);
    }

private:
    static gtsam::Vector3 predictVelocityBody(
        const gtsam::Pose3& pose,
        const gtsam::Vector3& velocity_world)
    {
        return pose.rotation().unrotate(velocity_world);
    }

    gtsam::Vector3 measured_velocity_body_;
};

void addPoseZFactor(
    gtsam::NonlinearFactorGraph& graph,
    gtsam::Key pose_key,
    gtsam::Key z_key,
    double sigma)
{
    const auto pose_z_noise = gtsam::noiseModel::Isotropic::Sigma(1, sigma);
    graph.add(PoseZFactor(pose_key, z_key, pose_z_noise));
}

void addDepthPriorFactor(
    gtsam::NonlinearFactorGraph& graph,
    gtsam::Key z_key,
    double measured_z,
    double sigma)
{
    const auto depth_noise =
        gtsam::noiseModel::Isotropic::Sigma(1, sigma);

    graph.add(gtsam::PriorFactor<double>(
        z_key,
        measured_z,
        depth_noise
    ));
}

void addDebugAttitudePriorFactor(
    gtsam::NonlinearFactorGraph& graph,
    gtsam::Key pose_key,
    const gtsam::Point3& translation,
    double sigma)
{
    const auto attitude_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << sigma, sigma, sigma, 1e6, 1e6, 1e6).finished()
    );

    graph.add(gtsam::PriorFactor<gtsam::Pose3>(
        pose_key,
        gtsam::Pose3(gtsam::Rot3::Identity(), translation),
        attitude_noise
    ));
}

std::string keyToString(gtsam::Key key)
{
    const gtsam::Symbol symbol(key);
    std::ostringstream stream;
    stream << symbol.chr() << symbol.index();
    return stream.str();
}

double yawFromPose(const gtsam::Pose3& pose)
{
    return pose.rotation().rpy().z();
}
}

GraphManager::GraphManager(rclcpp::Node* node)
: logger_(node != nullptr ? node->get_logger() : rclcpp::get_logger("GraphManager"))
{
    gtsam::ISAM2Params params;

    params.relinearizeThreshold = 0.01;
    params.relinearizeSkip = 1;

    isam2_ = gtsam::ISAM2(params);

    frame_index_ = 0;

    current_pose_key_ = gtsam::Symbol('x', 0);
    current_velocity_key_ = gtsam::Symbol('v',0);
    current_bias_key_ = gtsam::Symbol('b', 0);
    current_z_key_ = gtsam::Symbol('z', 0);

    current_pose_estimate_ = gtsam::Pose3::Identity();
    current_bias_estimate_ = gtsam::imuBias::ConstantBias();
    current_z_estimate_ = 0.0;
    last_depth_measurement_ = 0.0;
    depth_sigma_ = 0.05;
    pose_z_sigma_ = 0.20;
    initial_velocity_prior_sigma_ = 1.0;
    dvl_match_max_dt_ = 0.10;
    dvl_displacement_factor_enabled_ = true;
    dvl_displacement_sigma_ = 0.05;
    debug_attitude_prior_enabled_ = false;
    debug_attitude_prior_sigma_ = 0.02;
    has_depth_measurement_ = false;
    has_ground_truth_ = false;
    latest_ground_truth_position_ = gtsam::Point3(0.0, 0.0, 0.0);
    latest_dvl_matched_ = false;
    latest_dvl_time_diff_ = -1.0;
    latest_dvl_measured_ = gtsam::Vector3::Zero();
    latest_dvl_pose_key_ = current_pose_key_;
    latest_dvl_velocity_key_ = current_velocity_key_;
    latest_debug_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    current_nav_state_ = gtsam::NavState(
        gtsam::Pose3::Identity(),
        gtsam::Vector3::Zero()
    );
    current_velocity_estimate_ = gtsam::Vector3::Zero();

    if (node != nullptr)
    {
        depth_sigma_ = node->declare_parameter<double>(
            "depth_sigma",
            depth_sigma_);
        pose_z_sigma_ = node->declare_parameter<double>(
            "pose_z_sigma",
            pose_z_sigma_);
        initial_velocity_prior_sigma_ = node->declare_parameter<double>(
            "initial_velocity_prior_sigma",
            initial_velocity_prior_sigma_);
        dvl_match_max_dt_ = node->declare_parameter<double>(
            "dvl_match_max_dt",
            dvl_match_max_dt_);
        dvl_displacement_factor_enabled_ = node->declare_parameter<bool>(
            "dvl_displacement_factor_enabled",
            dvl_displacement_factor_enabled_);
        dvl_displacement_sigma_ = node->declare_parameter<double>(
            "dvl_displacement_sigma",
            dvl_displacement_sigma_);
        debug_attitude_prior_enabled_ = node->declare_parameter<bool>(
            "debug_attitude_prior_enabled",
            debug_attitude_prior_enabled_);
        debug_attitude_prior_sigma_ = node->declare_parameter<double>(
            "debug_attitude_prior_sigma",
            debug_attitude_prior_sigma_);

        const auto qos = rclcpp::QoS(10).transient_local();
        const std::string path_topic =
            node->declare_parameter<std::string>("path_topic", "/slam_path");
        const std::string debug_topic =
            node->declare_parameter<std::string>("slam_debug_topic", "/slam_debug");
        const std::string ground_truth_pose_topic =
            node->declare_parameter<std::string>(
                "ground_truth_pose_topic",
                "/ground_truth/pose");
        path_pub_ = node->create_publisher<nav_msgs::msg::Path>(path_topic, qos);
        debug_pub_ = node->create_publisher<std_msgs::msg::String>(
            debug_topic,
            rclcpp::QoS(20));
        ground_truth_sub_ =
            node->create_subscription<geometry_msgs::msg::PoseStamped>(
                ground_truth_pose_topic,
                rclcpp::QoS(20),
                std::bind(
                    &GraphManager::groundTruthCallback,
                    this,
                    std::placeholders::_1));
        path_msg_.header.frame_id = "map";
        RCLCPP_INFO(
            node->get_logger(),
            "Publishing SLAM path on %s with frame_id map; debug=%s gt=%s",
            path_topic.c_str(),
            debug_topic.c_str(),
            ground_truth_pose_topic.c_str());
        RCLCPP_INFO(
            node->get_logger(),
            "Depth/DVL config depth_sigma=%.3f pose_z_sigma=%.3f initial_velocity_sigma=%.3f dvl_match_max_dt=%.3f dvl_displacement=%s dvl_displacement_sigma=%.3f debug_attitude_prior=%s sigma=%.3f",
            depth_sigma_,
            pose_z_sigma_,
            initial_velocity_prior_sigma_,
            dvl_match_max_dt_,
            dvl_displacement_factor_enabled_ ? "true" : "false",
            dvl_displacement_sigma_,
            debug_attitude_prior_enabled_ ? "true" : "false",
            debug_attitude_prior_sigma_);
    }
}
void GraphManager::addPriorPose()
{
    const gtsam::Pose3 prior_pose = gtsam::Pose3::Identity();

    const auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01).finished()
    );

    graph_.add(gtsam::PriorFactor<gtsam::Pose3>(
        current_pose_key_,
        prior_pose,
        prior_noise
    ));

    new_values_.insert(current_pose_key_, prior_pose);
    new_values_.insert(current_z_key_, prior_pose.z());
    addPoseZFactor(graph_, current_pose_key_, current_z_key_, pose_z_sigma_);
}

void GraphManager::optimize()
{
    const size_t graph_size = graph_.size();

    const gtsam::Vector3 pre_dvl_predicted_body =
        current_pose_estimate_.rotation().unrotate(current_velocity_estimate_);
    gtsam::Vector3 pre_dvl_residual = gtsam::Vector3::Zero();
    if (latest_dvl_matched_)
    {
        pre_dvl_residual = pre_dvl_predicted_body - latest_dvl_measured_;
    }
    const gtsam::Point3 pre_position = current_pose_estimate_.translation();
    RCLCPP_INFO(
        logger_,
        "[STATE pre_optimize] pose=(%.3f, %.3f, %.3f) velocity=(%.3f, %.3f, %.3f) yaw=%.3f",
        pre_position.x(),
        pre_position.y(),
        pre_position.z(),
        current_velocity_estimate_.x(),
        current_velocity_estimate_.y(),
        current_velocity_estimate_.z(),
        yawFromPose(current_pose_estimate_));
    RCLCPP_INFO(
        logger_,
        "[DVL pre_optimize] dvl_matched=%s dvl_time_diff=%.6f dvl_measured=(%.3f, %.3f, %.3f) pose_key=%s vel_key=%s estimated_v_world=(%.3f, %.3f, %.3f) predicted_v_body=(%.3f, %.3f, %.3f) residual=(%.3f, %.3f, %.3f)",
        latest_dvl_matched_ ? "true" : "false",
        latest_dvl_time_diff_,
        latest_dvl_measured_.x(),
        latest_dvl_measured_.y(),
        latest_dvl_measured_.z(),
        keyToString(latest_dvl_pose_key_).c_str(),
        keyToString(latest_dvl_velocity_key_).c_str(),
        current_velocity_estimate_.x(),
        current_velocity_estimate_.y(),
        current_velocity_estimate_.z(),
        pre_dvl_predicted_body.x(),
        pre_dvl_predicted_body.y(),
        pre_dvl_predicted_body.z(),
        pre_dvl_residual.x(),
        pre_dvl_residual.y(),
        pre_dvl_residual.z());

    isam2_.update(graph_, new_values_);
    isam2_.update();

    const gtsam::Values result = isam2_.calculateEstimate();

    const gtsam::Pose3 raw_pose_estimate =
        result.at<gtsam::Pose3>(current_pose_key_);

    current_velocity_estimate_ =
        result.at<gtsam::Vector3>(current_velocity_key_);

    current_bias_estimate_ =
        result.at<gtsam::imuBias::ConstantBias>(current_bias_key_);

    current_z_estimate_ =
        result.at<double>(current_z_key_);

    const gtsam::Point3 raw_position = raw_pose_estimate.translation();
    current_pose_estimate_ = gtsam::Pose3(
        raw_pose_estimate.rotation(),
        gtsam::Point3(
            raw_position.x(),
            raw_position.y(),
            current_z_estimate_
        )
    );

    current_nav_state_ = gtsam::NavState(
        current_pose_estimate_,
        current_velocity_estimate_
    );
    
    const gtsam::Point3 position = current_pose_estimate_.translation();
    const double depth_residual =
        has_depth_measurement_
            ? current_z_estimate_ - last_depth_measurement_
            : 0.0;

    RCLCPP_INFO(
        logger_,
        "[STATE post_optimize] frame=%zu pose=(%.3f, %.3f, %.3f) raw_pose_z=%.3f z_state=%.3f velocity=(%.3f, %.3f, %.3f) yaw=%.3f depth_residual=%.3f graph_size=%zu",
        frame_index_,
        position.x(),
        position.y(),
        position.z(),
        raw_position.z(),
        current_z_estimate_,
        current_velocity_estimate_.x(),
        current_velocity_estimate_.y(),
        current_velocity_estimate_.z(),
        yawFromPose(current_pose_estimate_),
        depth_residual,
        graph_size
    );

    const gtsam::Vector3 post_dvl_predicted_body =
        current_pose_estimate_.rotation().unrotate(current_velocity_estimate_);
    gtsam::Vector3 post_dvl_residual = gtsam::Vector3::Zero();
    if (latest_dvl_matched_)
    {
        post_dvl_residual = post_dvl_predicted_body - latest_dvl_measured_;
    }
    RCLCPP_INFO(
        logger_,
        "[DVL post_optimize] dvl_matched=%s dvl_time_diff=%.6f dvl_measured=(%.3f, %.3f, %.3f) pose_key=%s vel_key=%s estimated_v_world=(%.3f, %.3f, %.3f) predicted_v_body=(%.3f, %.3f, %.3f) residual=(%.3f, %.3f, %.3f)",
        latest_dvl_matched_ ? "true" : "false",
        latest_dvl_time_diff_,
        latest_dvl_measured_.x(),
        latest_dvl_measured_.y(),
        latest_dvl_measured_.z(),
        keyToString(latest_dvl_pose_key_).c_str(),
        keyToString(latest_dvl_velocity_key_).c_str(),
        current_velocity_estimate_.x(),
        current_velocity_estimate_.y(),
        current_velocity_estimate_.z(),
        post_dvl_predicted_body.x(),
        post_dvl_predicted_body.y(),
        post_dvl_predicted_body.z(),
        post_dvl_residual.x(),
        post_dvl_residual.y(),
        post_dvl_residual.z());

    publishDebugLog(latest_debug_stamp_);

    graph_.resize(0);
    new_values_.clear();
}

void GraphManager::addBetweenPose()
{
    const gtsam::Key previous_pose_key = current_pose_key_;
    const gtsam::Key next_pose_key = gtsam::Symbol('x', frame_index_ + 1);

    const gtsam::Pose3 relative_pose(
        gtsam::Rot3::Identity(),
        gtsam::Point3(1.0, 0.0, 0.0)
    );

    const auto between_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.05, 0.05, 0.05).finished()
    );

    graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
        previous_pose_key,
        next_pose_key,
        relative_pose,
        between_noise
    ));

    const gtsam::Pose3 next_pose_initial =
        current_pose_estimate_.compose(relative_pose);

    new_values_.insert(next_pose_key, next_pose_initial);
    current_pose_key_ = next_pose_key;
    frame_index_++;
}

void GraphManager::addImuMeasurement(
    const gtsam::Vector3& acc,
    const gtsam::Vector3& gyro,
    double dt)
{
    imu_manager_.addMeasurement(acc, gyro, dt);
}

void GraphManager::addDepthMeasurement(
    gtsam::Key pose_key,
    gtsam::Key z_key,
    double depth)
{
    const double measured_z = -depth;
    addPoseZFactor(graph_, pose_key, z_key, pose_z_sigma_);
    addDepthPriorFactor(graph_, z_key, measured_z, depth_sigma_);

    last_depth_measurement_ = measured_z;
    has_depth_measurement_ = true;

    RCLCPP_INFO(
        logger_,
        "Depth factor pose_key=%s z_key=%s depth=%.3f measured_z=%.3f current_z=%.3f residual=%.3f graph_size=%zu",
        keyToString(pose_key).c_str(),
        keyToString(z_key).c_str(),
        depth,
        measured_z,
        current_z_estimate_,
        current_z_estimate_ - measured_z,
        graph_.size()
    );
}

void GraphManager::addDvlMeasurement(
    const gtsam::Key& pose_key,
    const gtsam::Key& velocity_key,
    const gtsam::Vector3& velocity_body)
{
    const auto dvl_noise = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector3(0.05, 0.05, 0.10)
    );

    graph_.add(DvlVelocityFactor(
        pose_key,
        velocity_key,
        velocity_body,
        dvl_noise
    ));
}

void GraphManager::addDvlDisplacementMeasurement(
    const gtsam::Key& pose_i,
    const gtsam::Key& pose_j,
    const gtsam::Vector3& velocity_body,
    double dt)
{
    if (!dvl_displacement_factor_enabled_ || dt <= 0.0)
    {
        return;
    }

    const gtsam::Vector3 displacement_body = velocity_body * dt;
    const gtsam::Pose3 measured_delta(
        gtsam::Rot3::Identity(),
        gtsam::Point3(
            displacement_body.x(),
            displacement_body.y(),
            displacement_body.z()));

    const auto displacement_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) <<
            1e6, 1e6, 1e6,
            dvl_displacement_sigma_,
            dvl_displacement_sigma_,
            dvl_displacement_sigma_).finished());

    graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
        pose_i,
        pose_j,
        measured_delta,
        displacement_noise));

    RCLCPP_INFO(
        logger_,
        "[DVL_DISP] pose_i=%s pose_j=%s dt=%.6f displacement_body=(%.3f, %.3f, %.3f) sigma=%.3f",
        keyToString(pose_i).c_str(),
        keyToString(pose_j).c_str(),
        dt,
        displacement_body.x(),
        displacement_body.y(),
        displacement_body.z(),
        dvl_displacement_sigma_);
}

bool GraphManager::processBuffers(
    std::deque<TimedImuMeasurement>& imu_buffer,
    std::deque<TimedDepthMeasurement>& depth_buffer,
    std::deque<TimedDvlMeasurement>& dvl_buffer,
    rclcpp::Time& last_processed_imu_stamp)
{
    if (imu_buffer.empty())
    {
        RCLCPP_WARN(logger_, "Skip processBuffers: IMU buffer is empty");
        return false;
    }

    if (depth_buffer.empty())
    {
        RCLCPP_WARN(logger_, "Skip processBuffers: depth buffer is empty");
        return false;
    }

    if (last_processed_imu_stamp.nanoseconds() == 0)
    {
        last_processed_imu_stamp = imu_buffer.front().stamp;
        imu_buffer.pop_front();
    }

    if (imu_buffer.empty())
    {
        RCLCPP_WARN(logger_, "Skip processBuffers: waiting for IMU samples after initial timestamp");
        return false;
    }

    const rclcpp::Time window_start = last_processed_imu_stamp;
    const rclcpp::Time window_end = imu_buffer.back().stamp;

    if (window_end <= window_start)
    {
        RCLCPP_WARN(
            logger_,
            "Skip processBuffers: non-increasing IMU window start=%ld end=%ld",
            window_start.nanoseconds(),
            window_end.nanoseconds());
        imu_buffer.clear();
        return false;
    }

    size_t imu_used = 0;
    while (!imu_buffer.empty())
    {
        const TimedImuMeasurement imu = imu_buffer.front();
        imu_buffer.pop_front();

        const double dt = (imu.stamp - last_processed_imu_stamp).seconds();
        if (dt <= 0.0)
        {
            RCLCPP_WARN(
                logger_,
                "Drop IMU sample with non-positive dt=%.9f stamp=%ld last=%ld",
                dt,
                imu.stamp.nanoseconds(),
                last_processed_imu_stamp.nanoseconds());
            continue;
        }

        addImuMeasurement(imu.acc, imu.gyro, dt);
        last_processed_imu_stamp = imu.stamp;
        ++imu_used;
    }

    if (imu_used == 0)
    {
        RCLCPP_WARN(logger_, "Skip processBuffers: no valid IMU samples in window");
        return false;
    }

    TimedDepthMeasurement matched_depth;
    int64_t best_dt_ns = std::numeric_limits<int64_t>::max();

    for (const auto& depth : depth_buffer)
    {
        const int64_t dt_ns =
            std::llabs((depth.stamp - last_processed_imu_stamp).nanoseconds());
        if (dt_ns < best_dt_ns)
        {
            best_dt_ns = dt_ns;
            matched_depth = depth;
        }
    }

    while (!depth_buffer.empty() &&
           depth_buffer.front().stamp < last_processed_imu_stamp)
    {
        depth_buffer.pop_front();
    }

    TimedDvlMeasurement matched_dvl;
    int64_t best_dvl_dt_ns = std::numeric_limits<int64_t>::max();
    bool dvl_matched = false;

    for (const auto& dvl : dvl_buffer)
    {
        const int64_t dt_ns =
            std::llabs((dvl.stamp - last_processed_imu_stamp).nanoseconds());
        if (dt_ns < best_dvl_dt_ns)
        {
            best_dvl_dt_ns = dt_ns;
            matched_dvl = dvl;
        }
    }

    const double dvl_time_diff =
        best_dvl_dt_ns == std::numeric_limits<int64_t>::max()
            ? -1.0
            : static_cast<double>(best_dvl_dt_ns) * 1e-9;
    dvl_matched =
        !dvl_buffer.empty() &&
        dvl_time_diff <= dvl_match_max_dt_;

    while (!dvl_buffer.empty() &&
           dvl_buffer.front().stamp < last_processed_imu_stamp)
    {
        dvl_buffer.pop_front();
    }

    const gtsam::Key pose_i_for_dvl = current_pose_key_;
    const double dvl_window_dt = (last_processed_imu_stamp - window_start).seconds();
    const ImuWindowKeys imu_keys = addImuFactor();
    latest_debug_stamp_ = last_processed_imu_stamp;
    latest_dvl_matched_ = dvl_matched;
    latest_dvl_time_diff_ = dvl_time_diff;
    latest_dvl_measured_ =
        dvl_matched ? matched_dvl.velocity_body : gtsam::Vector3::Zero();
    latest_dvl_pose_key_ = imu_keys.pose_j;
    latest_dvl_velocity_key_ = imu_keys.vel_j;

    addDepthMeasurement(imu_keys.pose_j, imu_keys.z_j, matched_depth.depth);

    if (dvl_matched)
    {
        addDvlMeasurement(
            imu_keys.pose_j,
            imu_keys.vel_j,
            matched_dvl.velocity_body);
        addDvlDisplacementMeasurement(
            pose_i_for_dvl,
            imu_keys.pose_j,
            matched_dvl.velocity_body,
            dvl_window_dt);
    }
    else if (best_dvl_dt_ns != std::numeric_limits<int64_t>::max())
    {
        RCLCPP_WARN(
            logger_,
            "[DVL] matched=false dt=%.6f threshold=%.6f exceeds threshold; skip DVL factor pose_key=%s vel_key=%s",
            dvl_time_diff,
            dvl_match_max_dt_,
            keyToString(imu_keys.pose_j).c_str(),
            keyToString(imu_keys.vel_j).c_str());
    }

    const gtsam::Vector3 predicted_body =
        current_pose_estimate_.rotation().unrotate(current_velocity_estimate_);
    gtsam::Vector3 dvl_residual = gtsam::Vector3::Zero();
    if (dvl_matched)
    {
        dvl_residual = predicted_body - matched_dvl.velocity_body;
    }
    const double current_yaw = yawFromPose(current_pose_estimate_);

    RCLCPP_INFO(
        logger_,
        "[DVL] matched=%s dt=%.6f threshold=%.6f meas=(%.3f, %.3f, %.3f) pose_j=%s vel_j=%s v_world=(%.3f, %.3f, %.3f) pred=(%.3f, %.3f, %.3f) residual=(%.3f, %.3f, %.3f) yaw=%.3f",
        dvl_matched ? "true" : "false",
        dvl_time_diff,
        dvl_match_max_dt_,
        dvl_matched ? matched_dvl.velocity_body.x() : 0.0,
        dvl_matched ? matched_dvl.velocity_body.y() : 0.0,
        dvl_matched ? matched_dvl.velocity_body.z() : 0.0,
        keyToString(imu_keys.pose_j).c_str(),
        keyToString(imu_keys.vel_j).c_str(),
        current_velocity_estimate_.x(),
        current_velocity_estimate_.y(),
        current_velocity_estimate_.z(),
        predicted_body.x(),
        predicted_body.y(),
        predicted_body.z(),
        dvl_residual.x(),
        dvl_residual.y(),
        dvl_residual.z(),
        current_yaw);

    RCLCPP_INFO(
        logger_,
        "processBuffers imu_used=%zu depth_matched=true dvl_matched=%s depth_dt=%.6f dvl_time_diff=%.6f dvl_match_threshold=%.6f dvl_measured=[%.3f, %.3f, %.3f] current_velocity=[%.3f, %.3f, %.3f] imu_window=[%ld, %ld] remaining_imu=%zu remaining_depth=%zu remaining_dvl=%zu graph_size=%zu",
        imu_used,
        dvl_matched ? "true" : "false",
        static_cast<double>(best_dt_ns) * 1e-9,
        dvl_time_diff,
        dvl_match_max_dt_,
        dvl_matched ? matched_dvl.velocity_body.x() : 0.0,
        dvl_matched ? matched_dvl.velocity_body.y() : 0.0,
        dvl_matched ? matched_dvl.velocity_body.z() : 0.0,
        current_velocity_estimate_.x(),
        current_velocity_estimate_.y(),
        current_velocity_estimate_.z(),
        window_start.nanoseconds(),
        last_processed_imu_stamp.nanoseconds(),
        imu_buffer.size(),
        depth_buffer.size(),
        dvl_buffer.size(),
        graph_.size());

    return true;
}

void GraphManager::printImuPreintegration() const
{
    imu_manager_.printPreintegration();
}

void GraphManager::groundTruthCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    latest_ground_truth_position_ = gtsam::Point3(
        msg->pose.position.x,
        msg->pose.position.y,
        msg->pose.position.z);
    latest_ground_truth_stamp_ = rclcpp::Time(msg->header.stamp);
    has_ground_truth_ = true;
}

void GraphManager::publishDebugLog(const rclcpp::Time& stamp)
{
    const gtsam::Point3 slam_position = current_pose_estimate_.translation();
    const gtsam::Vector3 predicted_body =
        current_pose_estimate_.rotation().unrotate(current_velocity_estimate_);
    gtsam::Vector3 dvl_residual = gtsam::Vector3::Zero();
    if (latest_dvl_matched_)
    {
        dvl_residual = predicted_body - latest_dvl_measured_;
    }

    const gtsam::Vector3 position_error =
        has_ground_truth_
            ? gtsam::Vector3(
                  slam_position.x() - latest_ground_truth_position_.x(),
                  slam_position.y() - latest_ground_truth_position_.y(),
                  slam_position.z() - latest_ground_truth_position_.z())
            : gtsam::Vector3::Zero();

    RCLCPP_INFO(
        logger_,
        "[GT] ground_truth=(%.3f, %.3f, %.3f) slam=(%.3f, %.3f, %.3f) position_error=(%.3f, %.3f, %.3f) gt_available=%s",
        has_ground_truth_ ? latest_ground_truth_position_.x() : 0.0,
        has_ground_truth_ ? latest_ground_truth_position_.y() : 0.0,
        has_ground_truth_ ? latest_ground_truth_position_.z() : 0.0,
        slam_position.x(),
        slam_position.y(),
        slam_position.z(),
        position_error.x(),
        position_error.y(),
        position_error.z(),
        has_ground_truth_ ? "true" : "false");

    std::ostringstream stream;
    stream
        << "time " << stamp.seconds()
        << " gt_x " << (has_ground_truth_ ? latest_ground_truth_position_.x() : 0.0)
        << " gt_y " << (has_ground_truth_ ? latest_ground_truth_position_.y() : 0.0)
        << " gt_z " << (has_ground_truth_ ? latest_ground_truth_position_.z() : 0.0)
        << " slam_x " << slam_position.x()
        << " slam_y " << slam_position.y()
        << " slam_z " << slam_position.z()
        << " err_x " << position_error.x()
        << " err_y " << position_error.y()
        << " err_z " << position_error.z()
        << " vel_world ["
        << current_velocity_estimate_.x() << ", "
        << current_velocity_estimate_.y() << ", "
        << current_velocity_estimate_.z() << "]"
        << " dvl_measured ["
        << latest_dvl_measured_.x() << ", "
        << latest_dvl_measured_.y() << ", "
        << latest_dvl_measured_.z() << "]"
        << " dvl_residual ["
        << dvl_residual.x() << ", "
        << dvl_residual.y() << ", "
        << dvl_residual.z() << "]"
        << " dvl_matched " << (latest_dvl_matched_ ? 1 : 0)
        << " dvl_time_diff " << latest_dvl_time_diff_
        << " dvl_residual_x " << dvl_residual.x()
        << " dvl_residual_y " << dvl_residual.y()
        << " dvl_residual_z " << dvl_residual.z();

    const std::string debug_line = stream.str();
    RCLCPP_INFO(logger_, "[SLAM_DEBUG] %s", debug_line.c_str());

    if (debug_pub_)
    {
        std_msgs::msg::String msg;
        msg.data = debug_line;
        debug_pub_->publish(msg);
    }
}

void GraphManager::addPriorBias()
{
    const auto bias_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) <<
            0.001, 0.001, 0.001,   // gyro bias
            0.01,  0.01,  0.01     // accel bias
        ).finished()
    );

    graph_.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
        current_bias_key_,
        current_bias_estimate_,
        bias_noise
    ));

    new_values_.insert(
        current_bias_key_,
        current_bias_estimate_
    );
}

void GraphManager::addPriorVelocity()
{
    const auto velocity_noise =
        gtsam::noiseModel::Diagonal::Sigmas(
            gtsam::Vector3(
                initial_velocity_prior_sigma_,
                initial_velocity_prior_sigma_,
                initial_velocity_prior_sigma_)
        );

    graph_.add(
        gtsam::PriorFactor<gtsam::Vector3>(
            current_velocity_key_,
            current_velocity_estimate_,
            velocity_noise
        )
    );

    new_values_.insert(
        current_velocity_key_,
        current_velocity_estimate_
    );
}

GraphManager::ImuWindowKeys GraphManager::addImuFactor()
{
    const gtsam::Key pose_i = current_pose_key_;
    const gtsam::Key vel_i = current_velocity_key_;
    const gtsam::Key bias_i = current_bias_key_;

    const gtsam::Key pose_j = gtsam::Symbol('x', frame_index_ + 1);
    const gtsam::Key vel_j = gtsam::Symbol('v', frame_index_ + 1);
    const gtsam::Key bias_j = gtsam::Symbol('b', frame_index_ + 1);
    const gtsam::Key z_j = gtsam::Symbol('z', frame_index_ + 1);

    graph_.add(gtsam::CombinedImuFactor(
        pose_i,
        vel_i,
        pose_j,
        vel_j,
        bias_i,
        bias_j,
        imu_manager_.preintegratedMeasurements()
    ));

    const gtsam::NavState predicted_state =
        imu_manager_.predict(current_nav_state_);

    const double initial_z =
        has_depth_measurement_
            ? last_depth_measurement_
            : predicted_state.pose().z();
    const gtsam::Point3 predicted_translation =
        predicted_state.pose().translation();
    const gtsam::Pose3 pose_j_initial(
        predicted_state.pose().rotation(),
        gtsam::Point3(
            predicted_translation.x(),
            predicted_translation.y(),
            initial_z
        )
    );

    new_values_.insert(
        pose_j,
        pose_j_initial
    );

    new_values_.insert(
        vel_j,
        predicted_state.velocity()
    );

    new_values_.insert(
        bias_j,
        current_bias_estimate_
    );

    new_values_.insert(
        z_j,
        initial_z
    );

    if (debug_attitude_prior_enabled_)
    {
        addDebugAttitudePriorFactor(
            graph_,
            pose_j,
            pose_j_initial.translation(),
            debug_attitude_prior_sigma_);
    }

    current_pose_key_ = pose_j;
    current_velocity_key_ = vel_j;
    current_bias_key_ = bias_j;
    current_z_key_ = z_j;

    frame_index_++;

    current_pose_estimate_ = pose_j_initial;
    current_velocity_estimate_ = predicted_state.velocity();
    current_z_estimate_ = initial_z;
    current_nav_state_ = gtsam::NavState(
        current_pose_estimate_,
        current_velocity_estimate_
    );

    imu_manager_.resetIntegrationAndSetBias(current_bias_estimate_);

    return ImuWindowKeys{
        pose_j,
        vel_j,
        bias_j,
        z_j
    };
}

void GraphManager::publishPath(const rclcpp::Time& stamp)
{
    if (!path_pub_)
    {
        return;
    }

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = "map";

    const gtsam::Point3 t = current_pose_estimate_.translation();
    const Eigen::Quaterniond q = current_pose_estimate_.rotation().toQuaternion();

    pose_msg.pose.position.x = t.x();
    pose_msg.pose.position.y = t.y();
    pose_msg.pose.position.z = t.z();
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    path_msg_.header.stamp = stamp;
    path_msg_.poses.push_back(pose_msg);
    path_pub_->publish(path_msg_);
}

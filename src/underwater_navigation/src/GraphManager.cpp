#include "underwater_navigation/GraphManager.hpp"

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/Vector.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>

namespace
{
double yawFromPose(const gtsam::Pose3& pose);

constexpr std::array<const char*, 4> kSonarNames = {
    "up",
    "down",
    "left",
    "right"
};

double wrapToPi(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

double predictPipeWallRange(
    const gtsam::Pose3& pose,
    const gtsam::Vector3& offset_body,
    const gtsam::Vector3& direction_body,
    double pipe_radius)
{
    const gtsam::Vector3 offset_world = pose.rotation().rotate(offset_body);
    const gtsam::Vector3 direction_world = pose.rotation().rotate(direction_body);
    const gtsam::Point3 position = pose.translation();

    const double py = position.y() + offset_world.y();
    const double pz = position.z() + offset_world.z();
    const double dy = direction_world.y();
    const double dz = direction_world.z();

    const double a = dy * dy + dz * dz;
    const double b = 2.0 * (py * dy + pz * dz);
    const double c = py * py + pz * pz - pipe_radius * pipe_radius;

    if (a < 1e-12)
    {
        return std::numeric_limits<double>::infinity();
    }

    const double discriminant = b * b - 4.0 * a * c;
    if (discriminant < 0.0)
    {
        return std::numeric_limits<double>::infinity();
    }

    const double sqrt_discriminant = std::sqrt(discriminant);
    const double root_1 = (-b - sqrt_discriminant) / (2.0 * a);
    const double root_2 = (-b + sqrt_discriminant) / (2.0 * a);

    double best_root = std::numeric_limits<double>::infinity();
    if (root_1 >= 0.0)
    {
        best_root = root_1;
    }
    if (root_2 >= 0.0)
    {
        best_root = std::min(best_root, root_2);
    }

    return best_root;
}

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

class MagYawFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
public:
    MagYawFactor(
        gtsam::Key pose_key,
        double measured_yaw,
        const gtsam::SharedNoiseModel& noise_model)
    : gtsam::NoiseModelFactor1<gtsam::Pose3>(noise_model, pose_key),
      measured_yaw_(measured_yaw)
    {
    }

    gtsam::Vector evaluateError(
        const gtsam::Pose3& pose,
        gtsam::OptionalMatrixType H_pose) const override
    {
        const auto error_function =
            [this](const gtsam::Pose3& p) {
                return (gtsam::Vector(1) <<
                    wrapToPi(yawFromPose(p) - measured_yaw_)).finished();
            };

        if (H_pose)
        {
            *H_pose = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
                error_function,
                pose);
        }

        return error_function(pose);
    }

private:
    double measured_yaw_;
};

class SonarRangeFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
public:
    SonarRangeFactor(
        gtsam::Key pose_key,
        double measured_range,
        double pipe_radius,
        const gtsam::Vector3& offset_body,
        const gtsam::Vector3& direction_body,
        const gtsam::SharedNoiseModel& noise_model)
    : gtsam::NoiseModelFactor1<gtsam::Pose3>(noise_model, pose_key),
      measured_range_(measured_range),
      pipe_radius_(pipe_radius),
      offset_body_(offset_body),
      direction_body_(direction_body)
    {
    }

    gtsam::Vector evaluateError(
        const gtsam::Pose3& pose,
        gtsam::OptionalMatrixType H_pose) const override
    {
        const auto error_function =
            [this](const gtsam::Pose3& p) {
                const double predicted_range = predictPipeWallRange(
                    p,
                    offset_body_,
                    direction_body_,
                    pipe_radius_);
                const double residual =
                    std::isfinite(predicted_range)
                        ? predicted_range - measured_range_
                        : 10.0;
                return (gtsam::Vector(1) << residual).finished();
            };

        if (H_pose)
        {
            *H_pose = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
                error_function,
                pose);
        }

        return error_function(pose);
    }

private:
    double measured_range_;
    double pipe_radius_;
    gtsam::Vector3 offset_body_;
    gtsam::Vector3 direction_body_;
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

gtsam::SharedNoiseModel createMagYawNoiseModel(
    double sigma,
    bool use_robust_loss)
{
    const auto base_noise =
        gtsam::noiseModel::Isotropic::Sigma(1, std::max(1e-6, sigma));

    if (!use_robust_loss)
    {
        return base_noise;
    }

    return gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(1.345),
        base_noise);
}

void addMagYawFactor(
    gtsam::NonlinearFactorGraph& graph,
    gtsam::Key pose_key,
    double measured_yaw,
    double sigma,
    bool use_robust_loss)
{
    graph.add(MagYawFactor(
        pose_key,
        measured_yaw,
        createMagYawNoiseModel(sigma, use_robust_loss)));
}

gtsam::SharedNoiseModel createSonarRangeNoiseModel(
    double sigma,
    bool use_robust_loss)
{
    const auto base_noise =
        gtsam::noiseModel::Isotropic::Sigma(1, std::max(1e-6, sigma));

    if (!use_robust_loss)
    {
        return base_noise;
    }

    return gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(1.345),
        base_noise);
}

void addSonarRangeFactor(
    gtsam::NonlinearFactorGraph& graph,
    gtsam::Key pose_key,
    const GraphManager::TimedSonarMeasurement& measurement,
    double pipe_radius,
    double sigma,
    bool use_robust_loss)
{
    graph.add(SonarRangeFactor(
        pose_key,
        measurement.range,
        pipe_radius,
        measurement.offset_body,
        measurement.direction_body,
        createSonarRangeNoiseModel(sigma, use_robust_loss)));
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

double yawFromMagneticField(
    const gtsam::Vector3& magnetic_field,
    double yaw_offset)
{
    // The current simulator publishes x=cos(yaw), y=-sin(yaw).
    // Convert that magnetic convention back to the navigation yaw sign.
    return wrapToPi(
        -std::atan2(magnetic_field.y(), magnetic_field.x()) + yaw_offset);
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
    mag_factor_enabled_ = false;
    mag_match_max_dt_ = 0.05;
    mag_yaw_noise_sigma_ = 0.10;
    mag_yaw_offset_ = 0.0;
    mag_use_robust_loss_ = true;
    sonar_factor_enabled_ = false;
    sonar_match_max_dt_ = 0.05;
    sonar_range_noise_sigma_ = 0.05;
    sonar_use_robust_loss_ = true;
    pipe_radius_ = 2.5;
    debug_attitude_prior_enabled_ = false;
    debug_attitude_prior_sigma_ = 0.02;
    has_depth_measurement_ = false;
    has_ground_truth_ = false;
    has_ground_truth_yaw_ = false;
    latest_ground_truth_position_ = gtsam::Point3(0.0, 0.0, 0.0);
    latest_ground_truth_yaw_ = 0.0;
    latest_dvl_matched_ = false;
    latest_dvl_time_diff_ = -1.0;
    latest_dvl_measured_ = gtsam::Vector3::Zero();
    latest_dvl_pose_key_ = current_pose_key_;
    latest_dvl_velocity_key_ = current_velocity_key_;
    latest_mag_matched_ = false;
    latest_mag_time_diff_ = -1.0;
    latest_mag_yaw_measured_ = 0.0;
    latest_mag_residual_ = 0.0;
    for (size_t i = 0; i < latest_sonar_.size(); ++i)
    {
        latest_sonar_[i] = LatestSonarDebug{
            kSonarNames[i],
            false,
            -1.0,
            0.0,
            0.0,
            gtsam::Vector3::Zero(),
            gtsam::Vector3::Zero()};
    }
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
        mag_factor_enabled_ = node->declare_parameter<bool>(
            "mag_factor_enabled",
            mag_factor_enabled_);
        mag_match_max_dt_ = node->declare_parameter<double>(
            "mag_match_max_dt",
            mag_match_max_dt_);
        mag_yaw_noise_sigma_ = node->declare_parameter<double>(
            "mag_yaw_noise_sigma",
            mag_yaw_noise_sigma_);
        mag_yaw_offset_ = node->declare_parameter<double>(
            "mag_yaw_offset",
            mag_yaw_offset_);
        mag_use_robust_loss_ = node->declare_parameter<bool>(
            "mag_use_robust_loss",
            mag_use_robust_loss_);
        sonar_factor_enabled_ = node->declare_parameter<bool>(
            "sonar_factor_enabled",
            sonar_factor_enabled_);
        sonar_match_max_dt_ = node->declare_parameter<double>(
            "sonar_match_max_dt",
            sonar_match_max_dt_);
        sonar_range_noise_sigma_ = node->declare_parameter<double>(
            "sonar_range_noise_sigma",
            sonar_range_noise_sigma_);
        sonar_use_robust_loss_ = node->declare_parameter<bool>(
            "sonar_use_robust_loss",
            sonar_use_robust_loss_);
        pipe_radius_ = node->declare_parameter<double>(
            "pipe_radius",
            pipe_radius_);
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
            "Depth/DVL/MAG/Sonar config depth_sigma=%.3f pose_z_sigma=%.3f initial_velocity_sigma=%.3f dvl_match_max_dt=%.3f dvl_displacement=%s dvl_displacement_sigma=%.3f mag_enabled=%s mag_match_max_dt=%.3f mag_yaw_noise_sigma=%.3f mag_yaw_offset=%.3f mag_robust=%s sonar_enabled=%s sonar_match_max_dt=%.3f sonar_sigma=%.3f sonar_robust=%s pipe_radius=%.3f debug_attitude_prior=%s sigma=%.3f",
            depth_sigma_,
            pose_z_sigma_,
            initial_velocity_prior_sigma_,
            dvl_match_max_dt_,
            dvl_displacement_factor_enabled_ ? "true" : "false",
            dvl_displacement_sigma_,
            mag_factor_enabled_ ? "true" : "false",
            mag_match_max_dt_,
            mag_yaw_noise_sigma_,
            mag_yaw_offset_,
            mag_use_robust_loss_ ? "true" : "false",
            sonar_factor_enabled_ ? "true" : "false",
            sonar_match_max_dt_,
            sonar_range_noise_sigma_,
            sonar_use_robust_loss_ ? "true" : "false",
            pipe_radius_,
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
    std::deque<TimedMagMeasurement>& mag_buffer,
    std::deque<TimedSonarMeasurement>& sonar_buffer,
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

    TimedMagMeasurement matched_mag;
    int64_t best_mag_dt_ns = std::numeric_limits<int64_t>::max();
    bool mag_matched = false;

    if (mag_factor_enabled_)
    {
        for (const auto& mag : mag_buffer)
        {
            const int64_t dt_ns =
                std::llabs((mag.stamp - last_processed_imu_stamp).nanoseconds());
            if (dt_ns < best_mag_dt_ns)
            {
                best_mag_dt_ns = dt_ns;
                matched_mag = mag;
            }
        }
    }

    const double mag_time_diff =
        best_mag_dt_ns == std::numeric_limits<int64_t>::max()
            ? -1.0
            : static_cast<double>(best_mag_dt_ns) * 1e-9;
    mag_matched =
        mag_factor_enabled_ &&
        !mag_buffer.empty() &&
        mag_time_diff <= mag_match_max_dt_;

    while (!mag_buffer.empty() &&
           mag_buffer.front().stamp < last_processed_imu_stamp)
    {
        mag_buffer.pop_front();
    }

    std::array<TimedSonarMeasurement, 4> matched_sonar;
    std::array<int64_t, 4> best_sonar_dt_ns;
    std::array<bool, 4> sonar_matched;
    best_sonar_dt_ns.fill(std::numeric_limits<int64_t>::max());
    sonar_matched.fill(false);

    if (sonar_factor_enabled_)
    {
        for (const auto& sonar : sonar_buffer)
        {
            if (!sonar.valid)
            {
                continue;
            }

            for (size_t i = 0; i < kSonarNames.size(); ++i)
            {
                if (sonar.name != kSonarNames[i])
                {
                    continue;
                }

                const int64_t dt_ns =
                    std::llabs((sonar.stamp - last_processed_imu_stamp).nanoseconds());
                if (dt_ns < best_sonar_dt_ns[i])
                {
                    best_sonar_dt_ns[i] = dt_ns;
                    matched_sonar[i] = sonar;
                }
            }
        }

        for (size_t i = 0; i < kSonarNames.size(); ++i)
        {
            sonar_matched[i] =
                best_sonar_dt_ns[i] != std::numeric_limits<int64_t>::max() &&
                static_cast<double>(best_sonar_dt_ns[i]) * 1e-9 <= sonar_match_max_dt_;
        }
    }

    while (!sonar_buffer.empty() &&
           sonar_buffer.front().stamp < last_processed_imu_stamp)
    {
        sonar_buffer.pop_front();
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
    latest_mag_matched_ = mag_matched;
    latest_mag_time_diff_ = mag_time_diff;
    latest_mag_yaw_measured_ = 0.0;
    latest_mag_residual_ = 0.0;
    for (size_t i = 0; i < latest_sonar_.size(); ++i)
    {
        latest_sonar_[i] = LatestSonarDebug{
            kSonarNames[i],
            false,
            best_sonar_dt_ns[i] == std::numeric_limits<int64_t>::max()
                ? -1.0
                : static_cast<double>(best_sonar_dt_ns[i]) * 1e-9,
            0.0,
            0.0,
            gtsam::Vector3::Zero(),
            gtsam::Vector3::Zero()};
    }

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

    if (mag_factor_enabled_)
    {
        const double yaw_pred = yawFromPose(current_pose_estimate_);
        if (mag_matched)
        {
            latest_mag_yaw_measured_ =
                yawFromMagneticField(matched_mag.magnetic_field, mag_yaw_offset_);
            latest_mag_residual_ =
                wrapToPi(yaw_pred - latest_mag_yaw_measured_);
            addMagYawFactor(
                graph_,
                imu_keys.pose_j,
                latest_mag_yaw_measured_,
                mag_yaw_noise_sigma_,
                mag_use_robust_loss_);
        }

        if ((frame_index_ % 10) == 0 || !mag_matched)
        {
            RCLCPP_INFO(
                logger_,
                "[MAG] matched=%s dt=%.6f threshold=%.6f yaw_meas=%.6f yaw_pred=%.6f residual=%.6f pose_j=%s",
                mag_matched ? "true" : "false",
                mag_time_diff,
                mag_match_max_dt_,
                latest_mag_yaw_measured_,
                yaw_pred,
                latest_mag_residual_,
                keyToString(imu_keys.pose_j).c_str());
        }
    }

    if (sonar_factor_enabled_)
    {
        for (size_t i = 0; i < kSonarNames.size(); ++i)
        {
            double residual = 0.0;
            double range = 0.0;
            if (sonar_matched[i])
            {
                range = matched_sonar[i].range;
                const double predicted_range = predictPipeWallRange(
                    current_pose_estimate_,
                    matched_sonar[i].offset_body,
                    matched_sonar[i].direction_body,
                    pipe_radius_);
                residual =
                    std::isfinite(predicted_range)
                        ? predicted_range - matched_sonar[i].range
                        : 10.0;

                if (std::isfinite(predicted_range))
                {
                    addSonarRangeFactor(
                        graph_,
                        imu_keys.pose_j,
                        matched_sonar[i],
                        pipe_radius_,
                        sonar_range_noise_sigma_,
                        sonar_use_robust_loss_);
                }
                else
                {
                    sonar_matched[i] = false;
                    RCLCPP_WARN(
                        logger_,
                        "[SONAR] %s predicted invalid range; skip factor pose_j=%s measured=%.3f",
                        kSonarNames[i],
                        keyToString(imu_keys.pose_j).c_str(),
                        matched_sonar[i].range);
                }
            }

            latest_sonar_[i] = LatestSonarDebug{
                kSonarNames[i],
                sonar_matched[i],
                best_sonar_dt_ns[i] == std::numeric_limits<int64_t>::max()
                    ? -1.0
                    : static_cast<double>(best_sonar_dt_ns[i]) * 1e-9,
                range,
                residual,
                sonar_matched[i] ? matched_sonar[i].offset_body : gtsam::Vector3::Zero(),
                sonar_matched[i] ? matched_sonar[i].direction_body : gtsam::Vector3::Zero()};
        }

        if ((frame_index_ % 10) == 0)
        {
            for (const auto& sonar : latest_sonar_)
            {
                RCLCPP_INFO(
                    logger_,
                    "[SONAR] %s matched=%s dt=%.6f threshold=%.6f range=%.3f residual=%.3f pose_j=%s",
                    sonar.name.c_str(),
                    sonar.matched ? "true" : "false",
                    sonar.time_diff,
                    sonar_match_max_dt_,
                    sonar.range,
                    sonar.residual,
                    keyToString(imu_keys.pose_j).c_str());
            }
        }
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
        "processBuffers imu_used=%zu depth_matched=true dvl_matched=%s mag_matched=%s sonar_matched=[%s,%s,%s,%s] depth_dt=%.6f dvl_time_diff=%.6f mag_time_diff=%.6f dvl_match_threshold=%.6f mag_match_threshold=%.6f sonar_match_threshold=%.6f dvl_measured=[%.3f, %.3f, %.3f] current_velocity=[%.3f, %.3f, %.3f] imu_window=[%ld, %ld] remaining_imu=%zu remaining_depth=%zu remaining_dvl=%zu remaining_mag=%zu remaining_sonar=%zu graph_size=%zu",
        imu_used,
        dvl_matched ? "true" : "false",
        mag_matched ? "true" : "false",
        latest_sonar_[0].matched ? "true" : "false",
        latest_sonar_[1].matched ? "true" : "false",
        latest_sonar_[2].matched ? "true" : "false",
        latest_sonar_[3].matched ? "true" : "false",
        static_cast<double>(best_dt_ns) * 1e-9,
        dvl_time_diff,
        mag_time_diff,
        dvl_match_max_dt_,
        mag_match_max_dt_,
        sonar_match_max_dt_,
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
        mag_buffer.size(),
        sonar_buffer.size(),
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
    latest_ground_truth_yaw_ = yawFromPose(gtsam::Pose3(
        gtsam::Rot3::Quaternion(
            msg->pose.orientation.w,
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z),
        latest_ground_truth_position_));
    latest_ground_truth_stamp_ = rclcpp::Time(msg->header.stamp);
    has_ground_truth_ = true;
    has_ground_truth_yaw_ = true;
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
    const double estimated_yaw = yawFromPose(current_pose_estimate_);
    const double yaw_error =
        has_ground_truth_yaw_
            ? wrapToPi(estimated_yaw - latest_ground_truth_yaw_)
            : 0.0;
    std::array<double, 4> sonar_debug_residuals;
    for (size_t i = 0; i < latest_sonar_.size(); ++i)
    {
        sonar_debug_residuals[i] = latest_sonar_[i].residual;
        if (latest_sonar_[i].matched)
        {
            const double predicted_range = predictPipeWallRange(
                current_pose_estimate_,
                latest_sonar_[i].offset_body,
                latest_sonar_[i].direction_body,
                pipe_radius_);
            sonar_debug_residuals[i] =
                std::isfinite(predicted_range)
                    ? predicted_range - latest_sonar_[i].range
                    : latest_sonar_[i].residual;
        }
    }

    RCLCPP_INFO(
        logger_,
        "[GT] ground_truth=(%.3f, %.3f, %.3f) slam=(%.3f, %.3f, %.3f) position_error=(%.3f, %.3f, %.3f) gt_yaw=%.3f slam_yaw=%.3f yaw_error=%.3f gt_available=%s",
        has_ground_truth_ ? latest_ground_truth_position_.x() : 0.0,
        has_ground_truth_ ? latest_ground_truth_position_.y() : 0.0,
        has_ground_truth_ ? latest_ground_truth_position_.z() : 0.0,
        slam_position.x(),
        slam_position.y(),
        slam_position.z(),
        position_error.x(),
        position_error.y(),
        position_error.z(),
        has_ground_truth_yaw_ ? latest_ground_truth_yaw_ : 0.0,
        estimated_yaw,
        yaw_error,
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
        << " dvl_residual_z " << dvl_residual.z()
        << " yaw_error " << yaw_error
        << " mag_residual " << latest_mag_residual_
        << " mag_matched " << (latest_mag_matched_ ? 1 : 0)
        << " mag_time_diff " << latest_mag_time_diff_
        << " mag_yaw_measured " << latest_mag_yaw_measured_
        << " sonar_up_matched " << (latest_sonar_[0].matched ? 1 : 0)
        << " sonar_down_matched " << (latest_sonar_[1].matched ? 1 : 0)
        << " sonar_left_matched " << (latest_sonar_[2].matched ? 1 : 0)
        << " sonar_right_matched " << (latest_sonar_[3].matched ? 1 : 0)
        << " sonar_up_residual " << sonar_debug_residuals[0]
        << " sonar_down_residual " << sonar_debug_residuals[1]
        << " sonar_left_residual " << sonar_debug_residuals[2]
        << " sonar_right_residual " << sonar_debug_residuals[3]
        << " sonar_up_range " << latest_sonar_[0].range
        << " sonar_down_range " << latest_sonar_[1].range
        << " sonar_left_range " << latest_sonar_[2].range
        << " sonar_right_range " << latest_sonar_[3].range;

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

#include "underwater_navigation/GraphManager.hpp"

#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Vector.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/base/Vector.h>

#include <iostream>

GraphManager::GraphManager()
{
    gtsam::ISAM2Params params;

    params.relinearizeThreshold = 0.01;
    params.relinearizeSkip = 1;

    isam2_ = gtsam::ISAM2(params);

    frame_index_ = 0;

    current_pose_key_ = gtsam::Symbol('x', 0);
    current_velocity_key_ = gtsam::Symbol('v',0);
    current_bias_key_ = gtsam::Symbol('b', 0);

    current_pose_estimate_ = gtsam::Pose3::Identity();

    current_nav_state_ = gtsam::NavState(
        gtsam::Pose3::Identity(),
        gtsam::Vector3::Zero()
    );
    current_velocity_estimate_ = gtsam::Vector3::Zero();
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
}

void GraphManager::optimize()
{
    isam2_.update(graph_, new_values_);
    isam2_.update();

    const gtsam::Values result = isam2_.calculateEstimate();

    current_nav_state_ =
    result.at<gtsam::NavState>(current_pose_key_);

    current_pose_estimate_ =
    current_nav_state_.pose();
    
    result.print("Current estimate:\n");

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

void GraphManager::printImuPreintegration() const
{
    imu_manager_.printPreintegration();
}

void GraphManager::addPriorNavState()
{
    const auto nav_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(9) << 
            0.01, 0.01, 0.01,   // rotation
            0.01, 0.01, 0.01,   // position
            0.01, 0.01, 0.01    // velocity
        ).finished()
    );

    graph_.add(gtsam::PriorFactor<gtsam::NavState>(
        current_pose_key_,
        current_nav_state_,
        nav_noise
    ));

    new_values_.insert(current_pose_key_, current_nav_state_);
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
        imu_manager_.currentBias(),
        bias_noise
    ));

    new_values_.insert(
        current_bias_key_,
        imu_manager_.currentBias()
    );
}

void GraphManager::addPriorVelocity()
{
    const auto velocity_noise =
        gtsam::noiseModel::Diagonal::Sigmas(
            gtsam::Vector3(0.01,0.01,0.01)
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
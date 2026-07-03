#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/NavState.h>

#include "underwater_navigation/imu/ImuFactorManager.hpp"

class GraphManager
{
public:

    GraphManager();

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
    void addImuFactor();
    void printImuPreintegration() const;

    const gtsam::imuBias::ConstantBias& currentBias() const;

private:

    gtsam::NonlinearFactorGraph graph_;

    gtsam::Values new_values_;

    gtsam::ISAM2 isam2_;

    gtsam::Key current_pose_key_;
    gtsam::Key current_velocity_key_;
    gtsam::Vector3 current_velocity_estimate_;
    gtsam::Key current_bias_key_;

    size_t frame_index_;

    gtsam::Pose3 current_pose_estimate_;
    gtsam::NavState current_nav_state_;

    ImuFactorManager imu_manager_;
};
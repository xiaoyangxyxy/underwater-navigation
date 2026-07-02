#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/geometry/Pose3.h>

#include "underwater_navigation/imu/ImuFactorManager.hpp"

class GraphManager
{
public:

    GraphManager();

    void addPriorPose();

    void addBetweenPose();

    void optimize();

    void addImuMeasurement(
        const gtsam::Vector3& acc,
        const gtsam::Vector3& gyro,
        double dt
    );

    void printImuPreintegration() const;

private:

    gtsam::NonlinearFactorGraph graph_;

    gtsam::Values new_values_;

    gtsam::ISAM2 isam2_;

    gtsam::Key current_pose_key_;

    size_t frame_index_;

    gtsam::Pose3 current_pose_estimate_;

    ImuFactorManager imu_manager_;
};
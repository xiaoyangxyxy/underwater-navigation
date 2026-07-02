#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/geometry/Pose3.h>

class GraphManager
{
public:

    GraphManager();
    void addPriorPose();
    void addBetweenPose();
    void optimize();

private:

    gtsam::NonlinearFactorGraph graph_;

    gtsam::Values new_values_;

    gtsam::ISAM2 isam2_;
    
    gtsam::Key current_pose_key_;

    size_t frame_index_;
};
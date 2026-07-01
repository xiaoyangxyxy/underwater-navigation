#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>

class GraphManager
{
public:

    GraphManager();

private:

    gtsam::NonlinearFactorGraph graph_;

    gtsam::Values new_values_;

    gtsam::ISAM2 isam2_;
};
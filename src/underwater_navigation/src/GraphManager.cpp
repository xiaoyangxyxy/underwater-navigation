#include "underwater_navigation/GraphManager.hpp"

GraphManager::GraphManager()
{
    gtsam::ISAM2Params params;

    params.relinearizeThreshold = 0.01;
    params.relinearizeSkip = 1;

    isam2_ = gtsam::ISAM2(params);
}
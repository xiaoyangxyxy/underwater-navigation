#include "underwater_navigation/GraphManager.hpp"

#include <iostream>

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

  std::cout << "underwater_navigation_node started." << std::endl;

GraphManager graph_manager;

  graph_manager.addPriorPose();

  graph_manager.addPriorVelocity();

  graph_manager.addPriorBias();

  for (int i = 0; i < 100; ++i)
  {
      graph_manager.addImuMeasurement(
          gtsam::Vector3(1.0, 0.0, 0.0),
          gtsam::Vector3(0.0, 0.0, 0.0),
          0.01
      );
  }

  graph_manager.printImuPreintegration();

  graph_manager.addImuFactor();

  graph_manager.optimize();


std::cout << "Graph optimization finished." << std::endl;

    return 0;
}
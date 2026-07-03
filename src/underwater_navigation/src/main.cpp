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
  graph_manager.optimize();

std::cout << "Graph optimization finished." << std::endl;

    return 0;
}
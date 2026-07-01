#include "underwater_navigation/GraphManager.hpp"

#include <iostream>

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    std::cout << "underwater_navigation_node started." << std::endl;

    GraphManager graph_manager;

    std::cout << "GraphManager created successfully." << std::endl;

    return 0;
}
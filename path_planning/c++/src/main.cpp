#include <iostream>
#include <vector>
#include <utility>
#include "../include/planner.hpp"

int main() {
    // Create a Planner object
    Planner planner;

    // Define start and goal positions
    std::pair<double, double> start = {0.0, 0.0};
    std::pair<double, double> goal = {10.0, 10.0};

    // Define obstacles
    std::vector<std::pair<double, double>> obstacles = {
        {3.0, 3.0},
        {5.0, 5.0},
        {7.0, 7.0}
    };

    // Set up the planner
    planner.set_obstacles(obstacles);
    planner.set_start_goal(start, goal);

    // Create a graph (this step might be necessary depending on how your planner is implemented)
    std::unordered_map<std::pair<double, double>, std::vector<std::pair<double, double>>, pair_hash> graph;
    // You would need to populate this graph based on your specific requirements

    planner.set_graph(graph);

    // Plan the path using A* algorithm
    std::vector<std::pair<double, double>> path = planner.a_star();

    // Print the resulting path
    std::cout << "Planned path:" << std::endl;
    for (const auto& point : path) {
        std::cout << "(" << point.first << ", " << point.second << ")" << std::endl;
    }

    return 0;
}

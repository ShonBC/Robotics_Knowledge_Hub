#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <vector>
#include <unordered_map>
#include <functional>

// Custom hash function for std::pair<double, double>
namespace std {
    template<>
    struct hash<pair<double, double>> {
        size_t operator()(const pair<double, double>& p) const {
            return hash<double>()(p.first) ^ hash<double>()(p.second);
        }
    };
}

class Planner {
private:
    std::unordered_map<std::pair<double, double>, std::vector<std::pair<double, double>>, std::hash<std::pair<double, double>>> graph;
    std::vector<std::pair<double, double>> obstacles;
    std::pair<double, double> start;
    std::pair<double, double> goal;

public:
    Planner() = default; // Explicit default constructor

    void set_graph(const std::unordered_map<std::pair<double, double>, std::vector<std::pair<double, double>>, std::hash<std::pair<double, double>>>& g);
    void set_obstacles(const std::vector<std::pair<double, double>>& obs);
    void set_start_goal(const std::pair<double, double>& s, const std::pair<double, double>& g);
    double heuristic(const std::pair<double, double>& a, const std::pair<double, double>& b);
    std::vector<std::pair<double, double>> a_star();
    std::pair<double, double> random_state();
    std::pair<double, double> steer(const std::pair<double, double>& from_point, const std::pair<double, double>& to_point, double step_size);
    bool collision_free(const std::pair<double, double>& from_point, const std::pair<double, double>& to_point);
};

#endif // PLANNER_HPP

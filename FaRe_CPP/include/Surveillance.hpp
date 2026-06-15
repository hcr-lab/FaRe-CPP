#ifndef SURVEILLANCE_HPP
#define SURVEILLANCE_HPP

#include <vector>
#include <map>
#include <chrono>
#include "Map.hpp"
#include "Scout.hpp"
#include "Config.hpp"

using Point = std::pair<int, int>;

struct SurveillanceGoal {
    int iteration;
    Point goal;
    double area;
    double orientation;
    std::vector<Point> frontiers;
    Grid graph;
};

struct SurveillanceResult {
    Point frontier;
    double area;
    Grid sub_graph;
    double ori;
};

class Surveillance {
public:
    Surveillance(const Grid& grid_map, int surveillance_range, uint8_t free_cells, 
                 uint8_t state, const YAMLData& yaml_data);
    
    // Main exploration function
    std::vector<SurveillanceGoal> setGoals(const Point& current_pos, uint8_t explored_value,
                                          uint8_t unexplored_value, int steps, 
                                          int frontier_drop_rate);
    
private:
    Grid grid_map;
    int surveillance_range;
    uint8_t free_cells;
    uint8_t state;
    YAMLData yaml_data;
    Scout scout;
    
    // Internal methods
    SurveillanceResult surveillance(int iteration, const std::vector<Point>& frontiers,
                                    const Grid& graph);
};

#endif // SURVEILLANCE_HPP

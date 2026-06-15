#ifndef SCOUT_HPP
#define SCOUT_HPP

#include <vector>
#include <cmath>
#include "Map.hpp"

using Point = std::pair<int, int>;

struct FrontierResult {
    Grid grid;
    double angle_rad;
};

class Scout {
public:
    Scout();
    
    // FOV computation and frontier detection
    FrontierResult computeFOV(const Grid& grid_map, const Point& start_pos, 
                             int radius, double fov_angle = 90.0);
    
    std::vector<Point> findFrontierCells(const Grid& grid_map, uint8_t explored_value,
                                         uint8_t unexplored_value, uint8_t obstacle_value = 0,
                                         int buffer_distance = 4);

private:
    const std::vector<double> base_angles = {0, 180, 270, 90};  // N, S, W, E
    const int angle_samples = 400;
};

#endif // SCOUT_HPP

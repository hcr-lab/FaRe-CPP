#include "Scout.hpp"
#include <iostream>
#include <algorithm>
#include <cstring>

Scout::Scout() {}

FrontierResult Scout::computeFOV(const Grid& grid_map, const Point& start_pos, 
                                 int radius, double fov_angle) {
    Grid best_grid = grid_map;
    double max_area = 0;
    double best_angle = 0;
    
    int rows = grid_map.size();
    int cols = grid_map[0].size();
    
    for (double base_angle : base_angles) {
        Grid grid = grid_map;
        
        // Convert angles to radians and create FOV sector
        double start_rad = (base_angle - fov_angle / 2.0) * M_PI / 180.0;
        double end_rad = (base_angle + fov_angle / 2.0) * M_PI / 180.0;
        
        // Create angle samples
        std::vector<double> angles;
        for (int i = 0; i < angle_samples; ++i) {
            double angle = start_rad + (end_rad - start_rad) * i / (angle_samples - 1);
            angles.push_back(angle);
        }
        
        int x_start = start_pos.first;
        int y_start = start_pos.second;
        
        // Ray casting for FOV computation
        for (double angle : angles) {
            for (int r = 1; r <= radius; ++r) {
                int x = static_cast<int>(x_start + r * std::cos(angle));
                int y = static_cast<int>(y_start + r * std::sin(angle));
                
                if (x >= 0 && x < cols && y >= 0 && y < rows) {
                    if (grid[y][x] == 0) {  // Occupied, block visibility
                        break;
                    } else if (grid[y][x] == 254) {  // Unoccupied, mark as explored
                        grid[y][x] = 150;
                    }
                } else {
                    break;
                }
            }
        }
        
        // Count explored area
        double explored_area = 0;
        for (const auto& row : grid) {
            for (uint8_t cell : row) {
                if (cell == 150) {
                    ++explored_area;
                }
            }
        }
        
        if (explored_area > max_area) {
            max_area = explored_area;
            best_grid = grid;
            best_angle = base_angle;
        }
    }
    
    return {best_grid, best_angle * M_PI / 180.0};
}

std::vector<Point> Scout::findFrontierCells(const Grid& grid_map, uint8_t explored_value,
                                           uint8_t unexplored_value, uint8_t obstacle_value,
                                           int buffer_distance) {
    std::vector<Point> frontier_cells;
    int rows = grid_map.size();
    int cols = grid_map[0].size();
    
    auto is_within_buffer = [&](const Point& pos) {
        for (int dx = -buffer_distance; dx <= buffer_distance; ++dx) {
            for (int dy = -buffer_distance; dy <= buffer_distance; ++dy) {
                int nx = pos.first + dx;
                int ny = pos.second + dy;
                if (nx >= 0 && nx < rows && ny >= 0 && ny < cols) {
                    if (grid_map[nx][ny] == obstacle_value) {
                        return true;
                    }
                }
            }
        }
        return false;
    };
    
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (grid_map[i][j] == unexplored_value) {
                // Check if adjacent to explored cell
                bool is_frontier = false;
                if ((i > 0 && grid_map[i-1][j] == explored_value) ||
                    (i < rows - 1 && grid_map[i+1][j] == explored_value) ||
                    (j > 0 && grid_map[i][j-1] == explored_value) ||
                    (j < cols - 1 && grid_map[i][j+1] == explored_value)) {
                    is_frontier = true;
                }
                
                if (is_frontier && !is_within_buffer({i, j})) {
                    frontier_cells.push_back({i, j});
                }
            }
        }
    }
    
    return frontier_cells;
}

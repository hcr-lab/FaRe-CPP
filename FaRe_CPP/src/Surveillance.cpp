#include "Surveillance.hpp"
#include "PathOptimizer.hpp"
#include <iostream>
#include <random>
#include <chrono>
#include <numeric>

Surveillance::Surveillance(const Grid& grid_map, int surveillance_range, uint8_t free_cells,
                          uint8_t state, const YAMLData& yaml_data)
    : grid_map(grid_map), surveillance_range(surveillance_range), free_cells(free_cells),
      state(state), yaml_data(yaml_data)
{
    std::cout << "Range: " << surveillance_range << std::endl;
}

SurveillanceResult Surveillance::surveillance(int iteration, const std::vector<Point>& frontiers,
                                             const Grid& graph) {
    double max_area = 0;
    Point selected_frontier = frontiers[0];  // Initialize to first frontier
    Grid best_graph = graph;
    double best_ori = 0;
    
    // Evaluate each frontier
    for (const auto& frontier : frontiers) {
        FrontierResult result = scout.computeFOV(graph, frontier, surveillance_range);
        
        // Count explored area
        double explored_area = 0;
        for (const auto& row : result.grid) {
            for (uint8_t cell : row) {
                if (cell == state) {
                    ++explored_area;
                }
            }
        }
        
        if (explored_area > max_area) {
            max_area = explored_area;
            selected_frontier = frontier;
            best_graph = result.grid;
            best_ori = result.angle_rad;
        }
    }
    
    return {selected_frontier, max_area, best_graph, best_ori};
}

std::vector<SurveillanceGoal> Surveillance::setGoals(const Point& current_pos, uint8_t explored_value,
                                                     uint8_t unexplored_value, int steps,
                                                     int frontier_drop_rate) {
    std::vector<SurveillanceGoal> area_goals;
    double total_area = 0;
    int iteration = 0;
    double t_time = 0;
    Grid graph = grid_map;
    
    std::random_device rd;
    std::mt19937 gen(rd());
    
    for (int i = 0; i < steps; ++i) {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        std::vector<Point> frontiers;
        if (i == 0) {
            frontiers = {current_pos};
        } else {
            frontiers = scout.findFrontierCells(graph, explored_value, unexplored_value);
        }
        
        if (i > 1 && !frontiers.empty()) {
            std::shuffle(frontiers.begin(), frontiers.end(), gen);
        }
        
        // Apply frontier dropout
        if (frontier_drop_rate > 0 && !frontiers.empty()) {
            std::vector<Point> filtered_frontiers;
            for (size_t idx = 0; idx < frontiers.size(); ++idx) {
                if (idx == 0 || (idx + 1) % frontier_drop_rate == 0) {
                    filtered_frontiers.push_back(frontiers[idx]);
                }
            }
            frontiers = filtered_frontiers;
        }
        
        if (frontiers.empty()) {
            std::cout << "No more frontiers found. Stopping exploration." << std::endl;
            break;
        }
        
        SurveillanceResult result = surveillance(iteration, frontiers, graph);
        total_area = result.area;
        iteration++;
        graph = result.sub_graph;
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
        t_time += duration.count();
        
        SurveillanceGoal goal;
        goal.iteration = iteration;
        goal.goal = result.frontier;
        goal.area = total_area;
        goal.orientation = result.ori;
        goal.frontiers = frontiers;
        goal.graph = graph;
        
        area_goals.push_back(goal);
        
        std::cout << "steps: " << iteration << " goal: (" << result.frontier.first << ", "
                  << result.frontier.second << ") ori: " << result.ori 
                  << " e_area: " << static_cast<int>(total_area) << " wp: " << frontiers.size()
                  << " e_time: " << duration.count() << "s t_time: " << static_cast<int>(t_time) << "s" << std::endl;
    }
    
    return area_goals;
}

#include "PathOptimizer.hpp"
#include <iostream>
#include <algorithm>
#include <random>

PathOptimizer::PathOptimizer(const std::vector<Point>& goals, double alpha, int max_iterations)
    : goals(goals), alpha(alpha), max_iterations(max_iterations), best_cost(std::numeric_limits<double>::infinity())
{
    std::random_device rd;
    rng.seed(rd());
}

double PathOptimizer::calculateDistance(const Point& p1, const Point& p2) {
    double dx = p1.first - p2.first;
    double dy = p1.second - p2.second;
    return std::sqrt(dx * dx + dy * dy);
}

double PathOptimizer::calculateTotalDistance(const std::vector<Point>& route) {
    double total = 0;
    for (size_t i = 0; i < route.size() - 1; ++i) {
        total += calculateDistance(route[i], route[i + 1]);
    }
    total += calculateDistance(route.back(), route.front());
    return total;
}

std::vector<Point> PathOptimizer::greedyRandomizedConstruction() {
    std::vector<Point> solution;
    std::vector<Point> remaining = goals;
    
    if (remaining.empty()) return solution;
    
    Point current_position = remaining[0];
    solution.push_back(current_position);
    remaining.erase(remaining.begin());
    
    while (!remaining.empty()) {
        // Calculate distances to all remaining goals
        std::vector<std::pair<double, Point>> distances;
        for (const auto& goal : remaining) {
            distances.push_back({calculateDistance(current_position, goal), goal});
        }
        
        // Sort by distance
        std::sort(distances.begin(), distances.end());
        
        // Select top candidates
        size_t num_candidates = std::max(size_t(1), static_cast<size_t>(alpha * distances.size()));
        std::uniform_int_distribution<> dis(0, num_candidates - 1);
        int selected_idx = dis(rng);
        
        Point next_goal = distances[selected_idx].second;
        solution.push_back(next_goal);
        
        // Remove from remaining
        auto it = std::find(remaining.begin(), remaining.end(), next_goal);
        if (it != remaining.end()) {
            remaining.erase(it);
        }
        
        current_position = next_goal;
    }
    
    // Return to start
    solution.push_back(goals[0]);
    
    return solution;
}

std::vector<Point> PathOptimizer::twoOptSwap(const std::vector<Point>& route, int i, int k) {
    std::vector<Point> new_route(route.begin(), route.begin() + i);
    new_route.insert(new_route.end(), route.begin() + i, route.begin() + k + 1);
    std::reverse(new_route.end() - (k - i + 1), new_route.end());
    new_route.insert(new_route.end(), route.begin() + k + 1, route.end());
    return new_route;
}

std::vector<Point> PathOptimizer::localSearch(const std::vector<Point>& solution) {
    std::vector<Point> best_route = solution;
    double best_cost_local = calculateTotalDistance(solution);
    
    bool improved = true;
    while (improved) {
        improved = false;
        for (size_t i = 1; i < best_route.size() - 2; ++i) {
            for (size_t k = i + 1; k < best_route.size(); ++k) {
                std::vector<Point> new_route = twoOptSwap(best_route, i, k);
                double new_cost = calculateTotalDistance(new_route);
                
                if (new_cost < best_cost_local) {
                    best_route = new_route;
                    best_cost_local = new_cost;
                    improved = true;
                }
            }
        }
    }
    
    return best_route;
}

std::vector<Point> PathOptimizer::run() {
    for (int iter = 0; iter < max_iterations; ++iter) {
        std::vector<Point> solution = greedyRandomizedConstruction();
        solution = localSearch(solution);
        double cost = calculateTotalDistance(solution);
        
        if (cost < best_cost) {
            best_solution = solution;
            best_cost = cost;
        }
    }
    
    return best_solution;
}

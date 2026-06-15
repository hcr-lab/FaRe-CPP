#include <iostream>
#include <fstream>
#include <iomanip>
#include "Config.hpp"
#include "Map.hpp"
#include "FOV.hpp"
#include "Scout.hpp"
#include "Surveillance.hpp"
#include "PathOptimizer.hpp"

void saveWaypoints(const std::string& output_dir, const std::vector<Point>& waypoints,
                   const std::vector<double>& orientations) {
    std::ofstream file(output_dir + "/wp_ori_data.txt");
    if (!file.is_open()) {
        std::cerr << "Error opening output file" << std::endl;
        return;
    }
    
    file << "wp = [";
    for (size_t i = 0; i < waypoints.size(); ++i) {
        if (i > 0) file << ", ";
        file << "(" << waypoints[i].first << ", " << waypoints[i].second << ")";
    }
    file << "]\n";
    
    file << "ori = [";
    for (size_t i = 0; i < orientations.size(); ++i) {
        if (i > 0) file << ", ";
        file << std::fixed << std::setprecision(6) << orientations[i];
    }
    file << "]\n";
    
    file.close();
}

double calculateRevisitTime(double linear_length, double cumulative_rotation,
                           double linear_speed = 0.3, double rotational_speed = 0.52) {
    double t_l = linear_length / linear_speed;
    double t_r = cumulative_rotation / rotational_speed;
    return t_l + t_r;
}

int main() {
    try {
        // Initialize configuration
        Config config;
        
        // Create output directory if it doesn't exist
        system(("mkdir -p " + config.output_dir).c_str());
        
        // Initialize map loader
        Map map_generator;
        
        // Load map and YAML
        std::cout << "Loading map and configuration..." << std::endl;
        Grid grid_map = map_generator.loadPGM(config.pgm_filename);
        YAMLData yaml_data = map_generator.loadYAML(config.yaml_filename);
        
        // Estimate total area
        double total_free_space = map_generator.estimateArea(grid_map, yaml_data, config.unexplored_value);
        std::cout << "Total free space: " << static_cast<int>(total_free_space) << " sq.mtrs" << std::endl;
        
        // Initialize surveillance
        std::cout << "Initializing surveillance..." << std::endl;
        Surveillance explore(grid_map, config.surveillance_range, config.unexplored_value,
                            config.state, yaml_data);
        
        // Generate waypoints
        std::cout << "Generating waypoints..." << std::endl;
        std::vector<SurveillanceGoal> goals = explore.setGoals(
            config.starting_position[0],
            config.explored_value,
            config.unexplored_value,
            config.steps,
            config.way_point_dropout
        );
        
        // Extract waypoints and orientations
        std::vector<Point> waypoints;
        std::vector<double> orientations;
        for (const auto& goal : goals) {
            waypoints.push_back(goal.goal);
            orientations.push_back(goal.orientation);
        }
        
        // Optimize path
        std::cout << "Optimizing path..." << std::endl;
        PathOptimizer optimizer(waypoints, config.optimizer_params.wp_threshold,
                               config.optimizer_params.num_iterations);
        std::vector<Point> optimized_wp = optimizer.run();
        
        // Save results
        std::cout << "Saving results..." << std::endl;
        saveWaypoints(config.output_dir, optimized_wp, orientations);
        
        // Calculate metrics
        double total_orientation = 0;
        for (double ori : orientations) {
            total_orientation += ori;
        }
        
        double path_distance = 0;
        for (size_t i = 0; i < optimized_wp.size() - 1; ++i) {
            double dx = optimized_wp[i+1].first - optimized_wp[i].first;
            double dy = optimized_wp[i+1].second - optimized_wp[i].second;
            path_distance += std::sqrt(dx * dx + dy * dy) * yaml_data.resolution;
        }
        
        double revisit_time = calculateRevisitTime(path_distance, total_orientation);
        
        std::cout << "\n========== Results ==========" << std::endl;
        std::cout << "Number of waypoints: " << optimized_wp.size() << std::endl;
        std::cout << "Path distance: " << std::fixed << std::setprecision(2) << path_distance << " m" << std::endl;
        std::cout << "Total rotation: " << std::fixed << std::setprecision(2) << total_orientation << " rad" << std::endl;
        std::cout << "Estimated revisit time: " << std::fixed << std::setprecision(2) << revisit_time << " s" << std::endl;
        std::cout << "=============================" << std::endl;
        
        std::cout << "\nResults saved to: " << config.output_dir << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}

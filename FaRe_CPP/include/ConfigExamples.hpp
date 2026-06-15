#ifndef CONFIG_EXAMPLE_HPP
#define CONFIG_EXAMPLE_HPP

#include <vector>
#include <string>

// Example configurations for different environments

// AWS RoboMaker Small House Configuration
struct HouseConfig {
    std::vector<std::pair<int, int>> starting_position = {{356, 33}};
    int explored_value = 150;
    int unexplored_value = 254;
    int state = 150;
    int steps = 25;
    int surveillance_range = 100;
    int way_point_dropout = 0;
    std::string output_dir = "./output/house";
    std::string pgm_filename = "/path/to/house/map.pgm";
    std::string yaml_filename = "/path/to/house/map.yaml";
    double wp_threshold = 0.3;
    int num_iterations = 5;
};

// Dynamic Logistics Warehouse Configuration
struct WarehouseConfig {
    std::vector<std::pair<int, int>> starting_position = {{100, 100}};
    int explored_value = 150;
    int unexplored_value = 254;
    int state = 150;
    int steps = 50;
    int surveillance_range = 120;
    int way_point_dropout = 0;
    std::string output_dir = "./output/warehouse";
    std::string pgm_filename = "/path/to/warehouse/map.pgm";
    std::string yaml_filename = "/path/to/warehouse/map.yaml";
    double wp_threshold = 0.3;
    int num_iterations = 5;
};

// Custom High-FOV Sensor Configuration
struct HighFOVConfig {
    std::vector<std::pair<int, int>> starting_position = {{500, 500}};
    int explored_value = 150;
    int unexplored_value = 254;
    int state = 150;
    int steps = 15;
    int surveillance_range = 150;  // Larger range
    int way_point_dropout = 2;
    std::string output_dir = "./output/high_fov";
    std::string pgm_filename = "/path/to/custom/map.pgm";
    std::string yaml_filename = "/path/to/custom/map.yaml";
    double wp_threshold = 0.5;     // More greedy
    int num_iterations = 10;        // More optimization iterations
};

// Low-power Robot Configuration (conservative)
struct LowPowerConfig {
    std::vector<std::pair<int, int>> starting_position = {{256, 256}};
    int explored_value = 150;
    int unexplored_value = 254;
    int state = 150;
    int steps = 10;
    int surveillance_range = 50;
    int way_point_dropout = 5;
    std::string output_dir = "./output/low_power";
    std::string pgm_filename = "/path/to/map.pgm";
    std::string yaml_filename = "/path/to/map.yaml";
    double wp_threshold = 0.2;     // Less greedy
    int num_iterations = 3;         // Fewer iterations
};

#endif // CONFIG_EXAMPLE_HPP

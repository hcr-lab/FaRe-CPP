#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <vector>
#include <string>
#include <map>

struct OptimizerParams {
    double wp_threshold;      // GRASP threshold
    int num_iterations;       // Number of iterations
};

struct Config {
    std::vector<std::pair<int, int>> starting_position;
    int explored_value;
    int unexplored_value;
    int state;
    int steps;
    int surveillance_range;
    int way_point_dropout;
    std::string output_dir;
    std::string pgm_filename;
    std::string yaml_filename;
    OptimizerParams optimizer_params;
    
    // Default constructor
    Config() 
        : explored_value(150)
        , unexplored_value(254)
        , state(150)
        , steps(25)
        , surveillance_range(100)
        , way_point_dropout(0)
        , output_dir("./output")
        , pgm_filename("./maps/map.pgm")
        , yaml_filename("./maps/map.yaml")
    {
        starting_position.push_back({375, 262});
        optimizer_params.wp_threshold = 0.3;
        optimizer_params.num_iterations = 5;
    }
};

#endif // CONFIG_HPP

#ifndef MAP_HPP
#define MAP_HPP

#include <vector>
#include <string>
#include <map>
#include <opencv2/opencv.hpp>

using Grid = std::vector<std::vector<uint8_t>>;

struct YAMLData {
    double resolution;
    std::vector<double> origin;
    int negate;
    double occupied_thresh;
    double free_thresh;
};

class Map {
public:
    Map();
    
    // Load and save functions
    Grid loadPGM(const std::string& pgm_path);
    YAMLData loadYAML(const std::string& yaml_path);
    void savePGM(const std::string& path, const Grid& grid);
    
    // Utility functions
    double estimateArea(const Grid& grid, const YAMLData& yaml_data, uint8_t free_cell_value);
    Grid convertToRGB(const Grid& grid, bool show_frontiers = false);
    Grid convertToBinary(const Grid& grid, uint8_t free_cell_value = 254);
    
    // Transformation functions
    Grid rotateArray90(const Grid& array);
    Grid flipArrayVertically(const Grid& array);

private:
    YAMLData parseYAML(const std::string& content);
};

#endif // MAP_HPP

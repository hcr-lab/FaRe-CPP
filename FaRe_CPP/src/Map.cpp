#include "Map.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>

Map::Map() {}

Grid Map::loadPGM(const std::string& pgm_path) {
    cv::Mat img = cv::imread(pgm_path, cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        throw std::runtime_error("Failed to load PGM file: " + pgm_path);
    }
    
    Grid grid(img.rows, std::vector<uint8_t>(img.cols));
    for (int i = 0; i < img.rows; ++i) {
        for (int j = 0; j < img.cols; ++j) {
            grid[i][j] = img.at<uint8_t>(i, j);
        }
    }
    
    std::cout << "Loaded PGM: " << img.rows << "x" << img.cols << std::endl;
    return grid;
}

YAMLData Map::loadYAML(const std::string& yaml_path) {
    std::ifstream file(yaml_path);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to load YAML file: " + yaml_path);
    }
    
    std::stringstream buffer;
    buffer << file.rdbuf();
    file.close();
    
    return parseYAML(buffer.str());
}

YAMLData Map::parseYAML(const std::string& content) {
    YAMLData data;
    data.resolution = 0.05;  // Default
    data.origin = {0, 0, 0};
    data.negate = 0;
    data.occupied_thresh = 0.65;
    data.free_thresh = 0.196;
    
    std::istringstream iss(content);
    std::string line;
    
    while (std::getline(iss, line)) {
        if (line.find("resolution:") != std::string::npos) {
            data.resolution = std::stod(line.substr(line.find(":") + 1));
        } else if (line.find("origin:") != std::string::npos) {
            // Parse origin array [x, y, z]
            size_t start = line.find("[");
            size_t end = line.find("]");
            if (start != std::string::npos && end != std::string::npos) {
                std::string origin_str = line.substr(start + 1, end - start - 1);
                std::istringstream origin_iss(origin_str);
                for (int i = 0; i < 3 && origin_iss; ++i) {
                    origin_iss >> data.origin[i];
                    if (origin_iss.peek() == ',') origin_iss.ignore();
                }
            }
        } else if (line.find("negate:") != std::string::npos) {
            data.negate = std::stoi(line.substr(line.find(":") + 1));
        } else if (line.find("occupied_thresh:") != std::string::npos) {
            data.occupied_thresh = std::stod(line.substr(line.find(":") + 1));
        } else if (line.find("free_thresh:") != std::string::npos) {
            data.free_thresh = std::stod(line.substr(line.find(":") + 1));
        }
    }
    
    return data;
}

void Map::savePGM(const std::string& path, const Grid& grid) {
    cv::Mat img(grid.size(), grid[0].size(), CV_8UC1);
    for (size_t i = 0; i < grid.size(); ++i) {
        for (size_t j = 0; j < grid[i].size(); ++j) {
            img.at<uint8_t>(i, j) = grid[i][j];
        }
    }
    cv::imwrite(path, img);
}

double Map::estimateArea(const Grid& grid, const YAMLData& yaml_data, uint8_t free_cell_value) {
    int unoccupied_cells = 0;
    for (const auto& row : grid) {
        for (uint8_t cell : row) {
            if (cell == free_cell_value) {
                ++unoccupied_cells;
            }
        }
    }
    
    double area_per_cell = yaml_data.resolution * yaml_data.resolution;
    return unoccupied_cells * area_per_cell;
}

Grid Map::convertToRGB(const Grid& grid, bool show_frontiers) {
    // For simplicity, convert to grayscale representation
    // In a real implementation, would create 3-channel RGB
    return grid;
}

Grid Map::convertToBinary(const Grid& grid, uint8_t free_cell_value) {
    Grid binary_grid = grid;
    for (auto& row : binary_grid) {
        for (auto& cell : row) {
            cell = (cell == free_cell_value) ? 255 : 0;
        }
    }
    return binary_grid;
}

Grid Map::rotateArray90(const Grid& array) {
    if (array.empty()) return array;
    
    int rows = array.size();
    int cols = array[0].size();
    Grid rotated(cols, std::vector<uint8_t>(rows));
    
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            rotated[cols - 1 - j][i] = array[i][j];
        }
    }
    
    return rotated;
}

Grid Map::flipArrayVertically(const Grid& array) {
    Grid flipped = array;
    std::reverse(flipped.begin(), flipped.end());
    return flipped;
}

#include "FOV.hpp"
#include <iostream>

FOV::FOV() : vertical_fov(0), horizontal_fov(0), radius(0) {
    calculateFOVandRadius();
}

void FOV::calculateFOVandRadius() {
    // 1. Calculate Vertical Field of View (FoV)
    double delta_y = b - a;
    vertical_fov = 2 * std::atan(delta_y / (2 * h)) * 180.0 / M_PI;
    
    // 2. Calculate Horizontal Field of View (FoV)
    double phi_half = std::atan(b / (2 * d2)) * 180.0 / M_PI;
    horizontal_fov = 2 * phi_half;
    
    // 3. Calculate Radius of the trapezoidal area
    radius = std::sqrt(d2 * d2 + (b / 2) * (b / 2));
    
    // Print results
    std::cout << "Vertical Field of View (FoV): " << vertical_fov << " degrees" << std::endl;
    std::cout << "Horizontal Field of View (FoV): " << horizontal_fov << " degrees" << std::endl;
    std::cout << "Radius of trapezoidal area: " << radius << " cm" << std::endl;
}

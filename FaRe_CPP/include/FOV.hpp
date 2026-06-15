#ifndef FOV_HPP
#define FOV_HPP

#include <vector>
#include <cmath>

class FOV {
public:
    FOV();
    
    // FOV calculation based on sensor specifications
    void calculateFOVandRadius();
    
    // Get FOV parameters
    double getVerticalFOV() const { return vertical_fov; }
    double getHorizontalFOV() const { return horizontal_fov; }
    double getRadius() const { return radius; }

private:
    double vertical_fov;      // Vertical field of view in degrees
    double horizontal_fov;    // Horizontal field of view in degrees
    double radius;            // Radius of the trapezoidal area in cm
    
    // Sensor specifications (in cm)
    const double h = 65.0;    // Height
    const double a = 70.0;    // Side length a
    const double b = 130.0;   // Side length b
    const double d1 = 15.0;   // Distance 1
    const double d2 = 115.0;  // Distance 2
};

#endif // FOV_HPP

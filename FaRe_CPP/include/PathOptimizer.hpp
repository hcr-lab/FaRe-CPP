#ifndef PATH_OPTIMIZER_HPP
#define PATH_OPTIMIZER_HPP

#include <vector>
#include <cmath>
#include <random>
#include <limits>

using Point = std::pair<int, int>;

class PathOptimizer {
public:
    PathOptimizer(const std::vector<Point>& goals, double alpha = 0.3, int max_iterations = 5);
    
    // Run the optimization
    std::vector<Point> run();
    
    // Get metrics
    double getTotalDistance() const { return best_cost; }
    
private:
    std::vector<Point> goals;
    double alpha;  // Randomization parameter (0-1)
    int max_iterations;
    std::vector<Point> best_solution;
    double best_cost;
    std::mt19937 rng;
    
    // Internal methods
    static double calculateDistance(const Point& p1, const Point& p2);
    double calculateTotalDistance(const std::vector<Point>& route);
    
    std::vector<Point> greedyRandomizedConstruction();
    std::vector<Point> twoOptSwap(const std::vector<Point>& route, int i, int k);
    std::vector<Point> localSearch(const std::vector<Point>& solution);
};

#endif // PATH_OPTIMIZER_HPP

# FaRe-CPP C++ Implementation

This is a C++ implementation of the FaRe-CPP algorithm for fast revisit coverage path planning for autonomous mobile robots.

## Prerequisites

### macOS
```bash
# Install dependencies using Homebrew
brew install cmake opencv yaml-cpp
```

### Linux (Ubuntu/Debian)
```bash
sudo apt-get install cmake libopencv-dev libyaml-cpp-dev
```

### Fedora
```bash
sudo dnf install cmake opencv-devel yaml-cpp-devel
```

## Building the Project

### 1. Create and enter build directory
```bash
cd FaRe_CPP
mkdir -p build
cd build
```

### 2. Configure with CMake
```bash
cmake ..
```

### 3. Build the project
```bash
make
```

The executable will be created at `bin/fare_cpp`

## Running the Program

### Quick start with build script
From the `FaRe_CPP` directory, run:
```bash
./build.sh
```

This will:
1. Create a build directory
2. Run CMake configuration
3. Compile the code
4. Run the executable

### Manual execution
```bash
./bin/fare_cpp
```

## Configuration

Edit `FaRe_CPP/include/Config.hpp` to customize:
- `starting_position`: Robot starting position
- `surveillance_range`: Sensor range in pixels
- `steps`: Number of exploration iterations
- `explored_value`: Grid cell value for explored areas (default: 150)
- `unexplored_value`: Grid cell value for free space (default: 254)
- `pgm_filename`: Path to occupancy grid map
- `yaml_filename`: Path to map metadata
- `output_dir`: Directory for output files

Or modify the default configuration values directly in `Config()` constructor.

## Output

The program generates:
- `wp_ori_data.txt`: Waypoints and orientations
- Console output with metrics:
  - Total waypoints
  - Path distance
  - Total rotation
  - Estimated revisit time

## Project Structure

```
FaRe_CPP/
├── CMakeLists.txt         # Build configuration
├── build.sh               # Build script
├── include/               # Header files
│   ├── Config.hpp         # Configuration structures
│   ├── Map.hpp            # Map loading and processing
│   ├── FOV.hpp            # Field of View computation
│   ├── Scout.hpp          # Scout and frontier detection
│   ├── PathOptimizer.hpp  # GRASP path optimization
│   └── Surveillance.hpp   # Main surveillance algorithm
├── src/                   # Implementation files
│   ├── Map.cpp
│   ├── FOV.cpp
│   ├── Scout.cpp
│   ├── PathOptimizer.cpp
│   ├── Surveillance.cpp
│   └── main.cpp           # Entry point
└── build/                 # Build artifacts (generated)
```

## Algorithm Overview

### Core Components

1. **Map Loading** (`Map.cpp`)
   - Loads PGM occupancy grids
   - Parses YAML metadata
   - Provides map transformations

2. **Field of View** (`FOV.cpp`)
   - Calculates sensor FOV based on specifications
   - Computes visible area from a position

3. **Scout** (`Scout.cpp`)
   - Performs FOV raycasting from given positions
   - Identifies frontier cells (exploration boundaries)
   - Selects best viewpoint based on explored area

4. **Path Optimization** (`PathOptimizer.cpp`)
   - Implements GRASP (Greedy Randomized Adaptive Search Procedure)
   - Uses 2-opt local search for improvement
   - Minimizes total path distance

5. **Surveillance** (`Surveillance.cpp`)
   - Orchestrates the exploration process
   - Iteratively finds and evaluates frontier points
   - Collects waypoints and orientations

### Algorithm Flow

1. Load occupancy grid and sensor configuration
2. Initialize exploration from starting position
3. Find frontier cells (boundary between explored/unexplored)
4. Evaluate each frontier with FOV computation
5. Select frontier with maximum explored area
6. Repeat until convergence or max iterations
7. Optimize waypoint order using GRASP
8. Output results and metrics

## Performance Notes

- **Computational Complexity**: O(n² × m) where n = grid size, m = iterations
- **Memory Usage**: O(n²) for grid storage
- **Multi-threading**: Ready for parallel frontier evaluation (ParallelFrontierEvaluation in Surveillance)

## Differences from Python Implementation

- **Type Safety**: Compile-time type checking
- **Performance**: ~10-100x faster execution
- **Memory Efficiency**: Direct memory management
- **Portability**: No runtime dependency on Python
- **Real-time**: Suitable for embedded robot systems

## Debugging

Enable verbose output by uncommenting debug statements in:
- `Scout.cpp`: FOV computation details
- `Surveillance.cpp`: Iteration progress
- `PathOptimizer.cpp`: Optimization iterations

Build with debug symbols:
```bash
cmake -DCMAKE_BUILD_TYPE=Debug ..
make
```

## License

Same as the Python implementation - see root repository LICENSE file.

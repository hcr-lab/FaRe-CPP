# Quick Start Guide - C++ FaRe-CPP

## Installation (First Time Setup)

### Step 1: Install Dependencies

**macOS:**
```bash
brew install cmake opencv yaml-cpp
```

**Ubuntu/Debian:**
```bash
sudo apt-get update
sudo apt-get install cmake libopencv-dev libyaml-cpp-dev
```

**Fedora/RHEL:**
```bash
sudo dnf install cmake opencv-devel yaml-cpp-devel
```

### Step 2: Make build script executable
```bash
cd FaRe_CPP
chmod +x build.sh
```

## Building the Project

### Automatic Build (Recommended)
```bash
cd FaRe_CPP
./build.sh
```

### Manual Build
```bash
cd FaRe_CPP
mkdir -p build
cd build
cmake ..
make
```

## Running the Program

### From FaRe_CPP directory:
```bash
./bin/fare_cpp
```

### Set custom paths:
Edit `FaRe_CPP/include/Config.hpp` and update:
```cpp
config.pgm_filename = "/path/to/your/map.pgm";
config.yaml_filename = "/path/to/your/map.yaml";
config.output_dir = "./output";
```

Then rebuild:
```bash
cd FaRe_CPP/build
cmake ..
make
```

## Expected Output

```
Loading map and configuration...
Loaded PGM: 1024x1024
Total free space: 85230 sq.mtrs
Initializing surveillance...
Range: 100
Generating waypoints...
steps: 1 goal: (356, 33) ori: 1.570796 e_area: 12450 wp: 45 e_time: 0s t_time: 0s
steps: 2 goal: (425, 78) ori: 3.141593 e_area: 24560 wp: 38 e_time: 1s t_time: 1s
...
Optimizing path...

========== Results ==========
Number of waypoints: 15
Path distance: 284.52 m
Total rotation: 47.12 rad
Estimated revisit time: 1024.34 s
=============================

Results saved to: ./output
```

## Configuration Options

### Basic Configuration
Edit `FaRe_CPP/include/Config.hpp`:

```cpp
config.steps = 25;                  // Iterations
config.surveillance_range = 100;    // Sensor range (pixels)
config.starting_position = {{356, 33}};
config.output_dir = "./output";
```

### Advanced Parameters
```cpp
config.explored_value = 150;        // Value for explored cells
config.unexplored_value = 254;      // Value for free space
config.state = 150;                 // Current state value
config.way_point_dropout = 0;       // Frontier filtering (0 = no filter)

// Optimization parameters
config.optimizer_params.wp_threshold = 0.3;    // GRASP alpha (0.0-1.0)
config.optimizer_params.num_iterations = 5;    // GRASP iterations
```

## File Structure

```
FaRe_CPP/
├── CMakeLists.txt          # Build system
├── build.sh                # Build script
├── README.md               # Full documentation
├── COMPARISON.md           # Python vs C++ comparison
├── QUICKSTART.md          # This file
├── include/               # Header files
│   ├── Config.hpp
│   ├── Map.hpp
│   ├── FOV.hpp
│   ├── Scout.hpp
│   ├── PathOptimizer.hpp
│   ├── Surveillance.hpp
│   └── ConfigExamples.hpp
├── src/                   # Implementation
│   ├── Map.cpp
│   ├── FOV.cpp
│   ├── Scout.cpp
│   ├── PathOptimizer.cpp
│   ├── Surveillance.cpp
│   └── main.cpp
├── build/                 # Build artifacts (auto-generated)
└── bin/                   # Compiled executable
    └── fare_cpp
```

## Troubleshooting

### Problem: CMake not found
```bash
# macOS
brew install cmake

# Ubuntu
sudo apt-get install cmake

# Fedora
sudo dnf install cmake
```

### Problem: opencv not found
```bash
# macOS
brew install opencv

# Ubuntu
sudo apt-get install libopencv-dev

# Fedora
sudo dnf install opencv-devel
```

### Problem: yaml-cpp not found
```bash
# macOS
brew install yaml-cpp

# Ubuntu
sudo apt-get install libyaml-cpp-dev

# Fedora
sudo dnf install yaml-cpp-devel
```

### Problem: Permission denied on build.sh
```bash
chmod +x build.sh
```

### Problem: Build errors
1. Clean build directory:
```bash
rm -rf build
mkdir -p build
```

2. Run CMake again:
```bash
cd build
cmake ..
make clean
make
```

### Problem: Map file not found
Ensure your PGM and YAML files exist at the specified paths in Config.hpp:
```bash
ls -la /path/to/your/map.pgm
ls -la /path/to/your/map.yaml
```

## Performance Tips

### For Faster Compilation
```bash
cd build
make -j$(nproc)  # Use all CPU cores
```

### For Faster Execution
- Use Release build (default)
- Ensure OpenCV is compiled with optimization
- Consider newer C++ standard: `-std=c++20`

### For Better Path Optimization
```cpp
config.optimizer_params.num_iterations = 10;  // More iterations
config.optimizer_params.wp_threshold = 0.5;   // More greedy
```

## Output Files

### wp_ori_data.txt
Contains optimized waypoints and orientations:
```
wp = [(356, 33), (425, 78), (489, 120), ...]
ori = [1.570796, 3.141593, 0.785398, ...]
```

### Console Output
- Iteration progress
- Explored area
- Processing times
- Final metrics

## Next Steps

1. **For Development**: See `README.md` for detailed documentation
2. **For Experimentation**: Check `include/ConfigExamples.hpp` for different configurations
3. **For Comparison**: Read `COMPARISON.md` for Python vs C++ details
4. **For ROS Integration**: Create a ROS wrapper calling the C++ library

## Common Workflows

### Process a Single Map
```bash
# 1. Update config
vim FaRe_CPP/include/Config.hpp

# 2. Rebuild
cd FaRe_CPP/build
cmake ..
make

# 3. Run
../bin/fare_cpp

# 4. Check results
cat output/wp_ori_data.txt
```

### Compare Python and C++ Results
```bash
# Run Python version
cd Cpp
python Surveillance.py

# Run C++ version
cd ../FaRe_CPP/build
cmake ..
make
./bin/fare_cpp

# Compare wp_ori_data.txt files
diff ../FaRe/wp_ori_data.txt ../output/wp_ori_data.txt
```

### Batch Processing Multiple Maps
Create a shell script:
```bash
#!/bin/bash
for map in maps/*.pgm; do
  base=$(basename "$map" .pgm)
  sed -i "s|pgm_filename = .*|pgm_filename = \"${map}\"|" include/Config.hpp
  sed -i "s|output_dir = .*|output_dir = \"./output/${base}\"|" include/Config.hpp
  cd build && cmake .. && make && cd ..
  ./bin/fare_cpp
done
```

## Performance Expectations

| Map Size | Execution Time | Expected Output |
|----------|---|---|
| 256×256 | ~50ms | Quick preview |
| 512×512 | ~150ms | Standard use |
| 1024×1024 | ~300ms | Large environment |
| 2048×2048 | ~1.2s | Industrial scale |

## Support

For issues or questions:
1. Check README.md for detailed documentation
2. Review COMPARISON.md for Python/C++ differences
3. Check error messages in console output
4. Verify file paths and configuration
5. Ensure all dependencies are installed

---

**Happy Exploring! 🤖**

cmake_minimum_required(VERSION 3.10)
project(FaRe_CPP)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(OpenCV REQUIRED)
find_package(YAML-CPP REQUIRED)
find_package(Threads REQUIRED)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIR})

# Source files
set(SOURCES
    src/Map.cpp
    src/FOV.cpp
    src/Scout.cpp
    src/PathOptimizer.cpp
    src/Surveillance.cpp
    src/main.cpp
)

# Create executable
add_executable(fare_cpp ${SOURCES})

# Link libraries
target_link_libraries(fare_cpp 
    ${OpenCV_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    Threads::Threads
)

# Set output directory
set_target_properties(fare_cpp PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
)

#!/bin/bash

# FaRe-CPP C++ Build Script

set -e  # Exit on error

echo "=== FaRe-CPP C++ Builder ==="
echo ""

# Check for required tools
if ! command -v cmake &> /dev/null; then
    echo "Error: CMake not found. Please install CMake."
    exit 1
fi

if ! command -v make &> /dev/null; then
    echo "Error: Make not found. Please install Make."
    exit 1
fi

# Create build directory if it doesn't exist
if [ ! -d "build" ]; then
    echo "Creating build directory..."
    mkdir -p build
fi

# Navigate to build directory
cd build

# Run CMake
echo "Running CMake configuration..."
cmake ..

# Compile
echo ""
echo "Compiling C++ code..."
make -j$(nproc)

echo ""
echo "=== Build Complete ==="
echo "Executable: ./bin/fare_cpp"
echo ""
echo "To run the program:"
echo "  cd .. && ./bin/fare_cpp"

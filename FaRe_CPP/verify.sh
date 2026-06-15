#!/bin/bash

# Verification script to check if all files are in place

echo "=== FaRe-CPP C++ Implementation Verification ==="
echo ""

# Define expected files
declare -a header_files=(
    "Config.hpp"
    "Map.hpp"
    "FOV.hpp"
    "Scout.hpp"
    "PathOptimizer.hpp"
    "Surveillance.hpp"
    "ConfigExamples.hpp"
)

declare -a source_files=(
    "Map.cpp"
    "FOV.cpp"
    "Scout.cpp"
    "PathOptimizer.cpp"
    "Surveillance.cpp"
    "main.cpp"
)

declare -a doc_files=(
    "README.md"
    "QUICKSTART.md"
    "COMPARISON.md"
    "IMPLEMENTATION_SUMMARY.md"
)

# Check header files
echo "Checking header files (include/)..."
for file in "${header_files[@]}"; do
    if [ -f "include/$file" ]; then
        echo "  ✓ $file"
    else
        echo "  ✗ $file (MISSING)"
    fi
done

echo ""
echo "Checking source files (src/)..."
for file in "${source_files[@]}"; do
    if [ -f "src/$file" ]; then
        echo "  ✓ $file"
    else
        echo "  ✗ $file (MISSING)"
    fi
done

echo ""
echo "Checking build files..."
if [ -f "CMakeLists.txt" ]; then
    echo "  ✓ CMakeLists.txt"
else
    echo "  ✗ CMakeLists.txt (MISSING)"
fi

if [ -f "build.sh" ]; then
    echo "  ✓ build.sh"
    if [ -x "build.sh" ]; then
        echo "    (executable: YES)"
    else
        echo "    (executable: NO - run 'chmod +x build.sh')"
    fi
else
    echo "  ✗ build.sh (MISSING)"
fi

echo ""
echo "Checking documentation files..."
for file in "${doc_files[@]}"; do
    if [ -f "$file" ]; then
        echo "  ✓ $file"
    else
        echo "  ✗ $file (MISSING)"
    fi
done

echo ""
echo "Checking dependencies..."

# Check for cmake
if command -v cmake &> /dev/null; then
    echo "  ✓ CMake: $(cmake --version | head -n1)"
else
    echo "  ✗ CMake: NOT INSTALLED"
fi

# Check for pkg-config or OpenCV
if pkg-config --modversion opencv4 2>/dev/null; then
    echo "  ✓ OpenCV: $(pkg-config --modversion opencv4)"
elif pkg-config --modversion opencv 2>/dev/null; then
    echo "  ✓ OpenCV: $(pkg-config --modversion opencv)"
else
    echo "  ✗ OpenCV: NOT FOUND"
fi

# Check for yaml-cpp
if pkg-config --modversion yaml-cpp 2>/dev/null; then
    echo "  ✓ YAML-CPP: $(pkg-config --modversion yaml-cpp)"
else
    echo "  ✗ YAML-CPP: NOT FOUND"
fi

# Check compiler
if command -v g++ &> /dev/null; then
    echo "  ✓ G++: $(g++ --version | head -n1)"
elif command -v clang++ &> /dev/null; then
    echo "  ✓ Clang++: $(clang++ --version | head -n1)"
else
    echo "  ✗ C++ Compiler: NOT FOUND"
fi

echo ""
echo "=== Summary ==="
total_files=$((${#header_files[@]} + ${#source_files[@]} + ${#doc_files[@]} + 2))
existing_files=0

# Count existing files
for file in "${header_files[@]}"; do
    if [ -f "include/$file" ]; then ((existing_files++)); fi
done
for file in "${source_files[@]}"; do
    if [ -f "src/$file" ]; then ((existing_files++)); fi
done
for file in "${doc_files[@]}"; do
    if [ -f "$file" ]; then ((existing_files++)); fi
done
if [ -f "CMakeLists.txt" ]; then ((existing_files++)); fi
if [ -f "build.sh" ]; then ((existing_files++)); fi

echo "Files: $existing_files/$total_files present"

echo ""
if [ $existing_files -eq $total_files ]; then
    echo "✓ All files present!"
    echo ""
    echo "Next steps:"
    echo "  1. Make build script executable: chmod +x build.sh"
    echo "  2. Build the project: ./build.sh"
    echo "  3. Run: ./bin/fare_cpp"
else
    echo "✗ Some files are missing!"
    echo "  Please check the file paths above."
fi

#!/bin/bash

# physics engine build script
# this script builds the physics engine and its examples

set -e # exit on error

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR=$PROJECT_DIR/build

echo "=== building physics-engine ==="
echo "project directory: $PROJECT_DIR"
echo "build directory: $BUILD_DIR"

# create a 'build' directory if it does not exist
if [ ! -d "$BUILD_DIR" ]; then
    echo "Creating build directory..."
    mkdir -p "$BUILD_DIR"
fi

cd "$BUILD_DIR"

if ! command -v cmake &> /dev/null; then
  echo "Error: CMake is not installed or not in PATH"
  echo "Please install CMake to build this project"
  echo ""
  echo "On macOS: brew install cmake"
  echo "On Ubuntu/Debian: sudo apt-get install cmake"
  echo "On Windows: winget install kitware.cmake"
  echo 1;
fi

# configure CMake
echo "=== Configuring CMake ==="
cmake .. -DCMAKE_BUILD_TYPE=Release

# build the project into 'build'
echo "=== Building ==="
if command -v nproc &> /dev/null; then
  # Linux
  JOBS=$(nproc)
elif command -v sysctl &> /dev/null; then
  # macOS
  JOBS=$(sysctl -n hw.ncpu)
else
  # Windows
  JOBS="$NUMBER_OF_PROCESSORS"
fi

echo "Building with $JOBS parallel jobs..."
cmake --build . --parallel "$JOBS"

echo "=== Build Complete ==="
echo ""
echo "Built the following executables:"
EXECUTABLES=("console_demo" "graphics_demo")

for exec in $EXECUTABLES
do
  if [ -f "$exec" ]; then
    echo "✓ $exec - Successful build"
  else
    echo "✗ $exec - Build failed"
  fi
done

echo ""
echo "To run the examples:"
echo "  cd $BUILD_DIR"
for exec in $EXECUTABLES;
do
  if [ -f "$exec" ]; then
    echo "  ./$exec"
  fi
done
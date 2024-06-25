#!/bin/bash

clear

# Specify the build directory (you can change this)
BUILD_DIR=build

# Create the build directory if it doesn't exist
mkdir -p $BUILD_DIR

# Navigate to the build directory
cd $BUILD_DIR

# Configure the project using CMake (modify the CMake options as needed)
cmake ..

# Build the project using make (adjust the number of CPU cores as needed)
make -j4

mv raytracing ..

# Return to the script's directory
cd ..

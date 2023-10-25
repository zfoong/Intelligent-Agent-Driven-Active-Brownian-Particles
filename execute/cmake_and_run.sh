#!/bin/bash

# Run CMake to generate build files
cmake -S . -B build

# Compile the code
cmake --build build

# Run the executable
python3 execute/run.py

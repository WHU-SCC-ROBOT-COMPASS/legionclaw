#!/bin/bash

# Clean script for perception modules
# Removes build directories, bin directories, and CMakeCache to fix stale path issues

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Cleaning perception build directories..."

# Clean all perception module build directories
find "$SCRIPT_DIR/modules/perception" -type d -name "build" -exec rm -rf {} + 2>/dev/null
find "$SCRIPT_DIR/modules/perception" -type d -name "log" -exec rm -rf {} + 2>/dev/null
find "$SCRIPT_DIR/modules/perception" -type f -name "CMakeCache.txt" -delete 2>/dev/null

# Clean binary files in bin directories (keep conf, json, yaml, md, etc.)
find "$SCRIPT_DIR/modules/perception" -path "*/bin/*" -type f ! -name "*.json" ! -name "*.conf" ! -name "*.yaml" ! -name "*.yml" ! -name "*.txt" ! -name "*.md" -delete 2>/dev/null

# Clean ros2 message build directory
if [ -d "$SCRIPT_DIR/modules/message/ros2/build" ]; then
    echo "Cleaning ros2 message build directory..."
    rm -rf "$SCRIPT_DIR/modules/message/ros2/build"
fi

# Clean any CMakeCache.txt in the entire project
find "$SCRIPT_DIR" -name "CMakeCache.txt" -delete 2>/dev/null
find "$SCRIPT_DIR" -name "CMakeFiles" -type d -exec rm -rf {} + 2>/dev/null

echo "Clean completed!"

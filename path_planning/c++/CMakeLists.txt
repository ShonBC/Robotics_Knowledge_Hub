cmake_minimum_required(VERSION 3.10)
project(PathPlanner)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(include)

# Add source files
set(SOURCES
    src/main.cpp
    src/planner.cpp
)

# Create executable
add_executable(path_planner ${SOURCES})

# If you have any external libraries, link them here
# For example:
# target_link_libraries(path_planner some_external_library)

# Set output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

cmake_minimum_required(VERSION 3.10)
project(SfM)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find packages
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Ceres REQUIRED)

# Include directories
include_directories(include ${OpenCV_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

# Add source files
file(GLOB_RECURSE SOURCES "src/*.cpp")

# Define the executable
add_executable(SfM ${SOURCES})

# Link libraries
target_link_libraries(SfM ${OpenCV_LIBS} Eigen3::Eigen ${CERES_LIBRARIES})

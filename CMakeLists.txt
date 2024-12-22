cmake_minimum_required(VERSION 3.10)
project(GenericBoatProject)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add the executable
add_executable(my_ompl_project examp.cpp generic-boat.cpp)

# Find OMPL package
find_package(ompl REQUIRED)

# Find Eigen package
find_package(Eigen3 REQUIRED)

# Include OMPL and Eigen directories
include_directories(${OMPL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

# Link OMPL libraries
target_link_libraries(my_ompl_project ${OMPL_LIBRARIES})
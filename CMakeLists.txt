cmake_minimum_required(VERSION 3.0.2)
project(ay_cpp)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
)


## The catkin_package macro generates cmake config files for your package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ay_cpp
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(ay_cpp
  src/optimizer.cpp
  include/ay_cpp/3rdparty/cma_es/cmaes.c
  include/ay_cpp/3rdparty/cma_es/boundary_transformation.c
)


## Install environment is not configured.

## Testing method is not configured.


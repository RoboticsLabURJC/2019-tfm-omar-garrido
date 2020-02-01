cmake_minimum_required(VERSION 2.8.3)
project(odometry_evaluation_file_creator)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  tf
  tf2
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        CATKIN_DEPENDS roscpp cv_bridge
#  INCLUDE_DIRS include
#  LIBRARIES odometry_evaluation_file_creator
#  CATKIN_DEPENDS cv_brdige roscpp tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include #include directory from the root folder where te Cmakefile is included
  ${catkin_INCLUDE_DIRS}
)

# MAIN PROGRAM TARGET CROSDIFODO
add_executable(odometry_evaluation_file_creator src/main.cpp src/OdometryEvaluationCreator.cpp include/OdometryEvaluationCreator.h)
target_link_libraries(odometry_evaluation_file_creator ${catkin_LIBRARIES}) #${OpenCV_LIBS})

cmake_minimum_required(VERSION 2.8.3)
project(rosified_difodo)

# Message to debug
message("STARTING BUILDING")

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
## Lo que se añada aqui se tiene que añadir tambien como:
## <build_depend>cv_bridge</build_depend>
## en el package.xml
## To see if the packages can be seen by your system before putting them here do:
## catkin_find "mypackage"
## catkin_find image_transport
## This will return the paths to that package if is on a catkin workspace
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  rospy
  std_msgs
  message_generation
  tf
)

## NON CATKIN OR ROS PACKAGES
## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)

# Ways to add modules:
# 1.(WORKS)
#find_package(mrpt-vision REQUIRED)
# 2. (DOESNT WORKS: Linking issue)
#find_package(mrpt REQUIRED COMPONENTS vision) #(DOESNT WORK I DUNNO WHY)
#message(STATUS "mrpt-vision_VERSION: ${mrpt-vision_VERSION}")
# 3.
# Find all MRPT libraries:
#find_package(MRPT 1.9.9 REQUIRED COMPONENTS poses gui vision)
find_package(MRPT REQUIRED COMPONENTS poses gui vision)
message(STATUS "MRPT_VERSION: ${MRPT_VERSION}")
message(STATUS "MRPT_LIBRARIES: ${MRPT_LIBRARIES}")


## Declare ROS messages and services

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
# The catkin packages must also be declared on the package.xml specifing wheter they are needed for build(compile time)
#or runtime (execution time):
#<build_depend>cv_bridge</build_depend>
#<build_export_depend>cv_bridge</build_export_depend>
#<exec_depend>cv_bridge</exec_depend>
catkin_package(
  CATKIN_DEPENDS roscpp cv_bridge image_transport rospy std_msgs)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
# See: https://stackoverflow.com/questions/13703647/how-to-properly-add-include-directories-with-cmake
include_directories(
        include #include directory from the root folder where te Cmakefile is included
        ${catkin_INCLUDE_DIRS}
)
# Another more specific target include
#target_include_directories(realsense_listener PUBLIC include)

# MAIN PROGRAM TARGET CROSDIFODO
add_executable(ros_difodo src/main.cpp src/CROSDifodo.cpp)
target_link_libraries(ros_difodo ${catkin_LIBRARIES} ${OpenCV_LIBS} ${MRPT_LIBRARIES})

#include <iostream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include "CROSDifodo.h"


int main(int argc, char **argv) {
    ROS_INFO_STREAM("ROSify version of Difodo algorithm starts");

    /* ROS DEFINITIONS */
    std::string node_name = "ros_difodo";
    ros::init(argc, argv, node_name);

    CROSDifodo ros_difodo = CROSDifodo();
    ros_difodo.loadConfiguration();
    ros_difodo.start();

    // ROS SPIN MANAGEMENT
    //http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
    ros::spin(); // Will stop the loop when press Ctrl+D in the terminal.

    ros_difodo.cancel();
    ROS_INFO_STREAM("Exiting successfully");
}
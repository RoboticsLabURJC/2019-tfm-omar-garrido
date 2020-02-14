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

    if (argc >= 2)
        ROS_INFO_STREAM("More than the needed configuration parameters were provided");

    for (int i=0; i < argc; i++) {
        std::cout << argv[i] << std::endl;
    }

    if (std::string(argv[1]) == "--use_inner_config")
    {
        ROS_INFO_STREAM("Using internal rosify_difodo config instead of launch file");
        ros_difodo.loadInnerConfiguration();
    } else {
        ROS_INFO_STREAM("Using launch file config instead of default inner config");
        ros_difodo.loadConfiguration();
    }

    ros_difodo.start();

    // ROS SPIN MANAGEMENT
    //http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
    ros::spin(); // Will stop the loop when press Ctrl+D in the terminal.

    ros_difodo.cancel();
    ROS_INFO_STREAM("Exiting successfully");
}

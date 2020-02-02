#include <iostream>
#include <ros/ros.h>
#include "OdometryEvaluationCreator.h"


int main(int argc, char **argv) {
    ROS_INFO_STREAM("Odometry Evaluator File Creator starts");

    /* ROS DEFINITIONS */
    std::string node_name = "odometry_evaluation_file_creator";
    ros::init(argc, argv, node_name);

    OdometryEvaluationCreator odom_eval_creator;

    if (argc > 3)
        ROS_INFO_STREAM("More than the needed configuration parameters were provided");

    if (std::string(argv[1]) == "--config_file_path")
    {
        ROS_INFO_STREAM("Using \"" << std::string(argv[2]) << "\" instead of launch file");
        odom_eval_creator.loadConfiguration(true, std::string(argv[2]));
    } else {
        ROS_INFO_STREAM("Using configuration within parameter server instead of local file");
        odom_eval_creator.loadConfiguration(false, "");
    }

    // ROS SPIN MANAGEMENT
    //http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
    ros::spin(); // Will stop the loop when press Ctrl+D in the terminal.
    ROS_INFO_STREAM("Exiting successfully");
}


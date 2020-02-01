#include <iostream>
#include <ros/ros.h>
#include "OdometryEvaluationCreator.h"


void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    ROS_INFO_STREAM("MAIN LLEGA ODOMETRIA");
}

int main(int argc, char **argv) {
    ROS_INFO_STREAM("Odometry Evaluator File Creation starts");

    /* ROS DEFINITIONS */
    std::string node_name = "odometry_evaluation_file_creator";
    ros::init(argc, argv, node_name);

//    ros::NodeHandle n;
//    ros::Subscriber odometry_sub = n.subscribe("/difodo/odometry",
//                                                     1000,
//                                                     odometryCallback);


    OdometryEvaluationCreator odom_eval_creator;
    odom_eval_creator.loadConfiguration();

//    if (argc >= 2)
//        ROS_INFO_STREAM("More than the needed configuration parameters were provided");
//
//    for (int i=0; i < argc; i++) {
//        std::cout << argv[i] << std::endl;
//    }
//
//    if (std::string(argv[1]) == "--use_inner_config")
//    {
//        ROS_INFO_STREAM("Using internal rosify_difodo config instead of launch file");
//        ros_difodo.loadInnerConfiguration();
//    } else {
//        ROS_INFO_STREAM("Using internal rosify_difodo config instead of launch file");
//        ros_difodo.loadConfiguration();
//    }

    // ROS SPIN MANAGEMENT
    //http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
    ros::spin(); // Will stop the loop when press Ctrl+D in the terminal.
    ROS_INFO_STREAM("Exiting successfully");
}


//
// Created by omar on 1/2/20.
//

#ifndef ODOMETRY_EVALUATION_FILE_CREATOR_ODOMETRYEVALUATIONCREATOR_H
#define ODOMETRY_EVALUATION_FILE_CREATOR_ODOMETRYEVALUATIONCREATOR_H

#include <ros/ros.h>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class OdometryEvaluationCreator {
public:
    OdometryEvaluationCreator();
    ~OdometryEvaluationCreator();
    void loadConfiguration(bool use_local_config, std::string config_file_path);

    ros::NodeHandle n;
    // Load parameters from YAML or parameter server is created with a launch.
    std::string odometry_topic = "/difodo/odometry";
    // The frame from where we want to convert
    std::string source_frame = "";
    // The frame that we wont to transform to
    std::string target_frame = "";
    // Do this parameters default but also allow to change for another one.
    std::string output_file_dir = "";

    std::ofstream output_file;

    // Is needed to be alive through the entire live of the object in order to get the messages
    ros::Subscriber odometry_sub;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener* tfListener;

    geometry_msgs::TransformStamped transformStamped;

    ros::Time odom_msg_time;

    bool is_first_time;

private:
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void writePoseToFile(double tx, double ty, double tz, double qx,  double qy, double qz,  double qw);
    void readConfigurationYAML(std::string filename);
    void readConfigurationFromParameterServer();

};


#endif //ODOMETRY_EVALUATION_FILE_CREATOR_ODOMETRYEVALUATIONCREATOR_H

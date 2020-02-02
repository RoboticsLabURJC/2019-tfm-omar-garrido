//
// Created by omar on 1/2/20.
//

#include "OdometryEvaluationCreator.h"

OdometryEvaluationCreator::OdometryEvaluationCreator() {
    this->odometry_topic = "/difodo/odometry";
    this->source_frame = "openni_camera";
    this->target_frame = "world";
    this->output_file_dir = "";
    this->is_first_time = true;
}

OdometryEvaluationCreator::~OdometryEvaluationCreator() {
    if (output_file.is_open()) output_file.close();
}

void OdometryEvaluationCreator::loadConfiguration(bool use_local_config, std::string config_file_path) {
    // Read configuration parameters
    if (use_local_config) {
        this->readConfigurationYAML(config_file_path);
    } else {
        // Use the configuration that is on the parameters server (using launch files)
        //TODO
    }

    // Output file creation
    time_t now = time(NULL);
    struct tm tstruct;
    char buf[40];
    tstruct = *localtime(&now);
    //format: day DD-MM-YYYY
    strftime(buf, sizeof(buf), "%d_%m_%Y-%H_%M_%S", &tstruct);
    std::string filename = std::string(buf) + "_odometry_evaluation_output.txt";

    std::string file_path;
    if (this->output_file_dir != "") {
        file_path = this->output_file_dir + "/" + filename;
    } else {
        file_path = filename;
    }
    output_file.open(file_path);
    ROS_INFO_STREAM("Output file has been created on: " << file_path);

    if (!output_file) {
        ROS_ERROR_STREAM("Output file could not be created");
        exit(-1);
    }

    // INITIALIZATION
    this->odometry_sub = this->n.subscribe(this->odometry_topic,
                                                     1000,
                                                     &OdometryEvaluationCreator::odometryCallback,
                                                     this);

    this->tfListener = new tf2_ros::TransformListener(this->tfBuffer);
}

void OdometryEvaluationCreator::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    ROS_INFO_STREAM("Processing odometry message with timestamp " << msg->header.stamp);
    this->odom_msg_time = msg->header.stamp;

    // We only want to take the first transformation, either is the moment where the odometry started or is a tf_static
    // so it will be the same tf repeated over time, so no need to pool constanly for it
    if (this->is_first_time) {
        try {
            // We get the transformation between frames. (world to odom frames)
            this->transformStamped = tfBuffer.lookupTransform(this->target_frame, this->source_frame,
                                                              ros::Time(0), ros::Duration(0,100000));
            is_first_time = false;
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ROS_ERROR("One of the frames specified seem not to be found on the /tf topic.");
            exit(-1);
        }
    }

    geometry_msgs::Pose pose_transformed;
    //TODO: Check that there is transformStamped (null or something)
    //Apply transformation to pose_stamped_in and get the pose transform on pose_transformed
    tf2::doTransform(msg->pose.pose, pose_transformed, this->transformStamped);

    this->writePoseToFile(pose_transformed.position.x, pose_transformed.position.y, pose_transformed.position.z,
                          pose_transformed.orientation.x, pose_transformed.orientation.y, pose_transformed.orientation.z,
                          pose_transformed.orientation.w);
}

void OdometryEvaluationCreator::writePoseToFile(double tx, double ty, double tz, double qx,  double qy, double qz,  double qw) {
    if (output_file.is_open())
    {
        //https://www.quora.com/How-do-I-customize-cout-in-C++-to-print-n-digit-after-the-decimal-point-of-a-floating-point
        output_file << std::fixed <<  std::setprecision(4) \
                        << this->odom_msg_time.toSec() << " " \
                        << tx << " " \
                        << ty << " " \
                        << tz << " " \
                        << qx << " " \
                        << qy << " " \
                        << qz << " " \
                        << qw << " " \
                        << std::endl;
    }
}

void OdometryEvaluationCreator::readConfigurationYAML(std::string filename) {
    cv::FileStorage fs;

    try {
        // Read config file
        fs.open(filename.c_str(), cv::FileStorage::READ);
        if (!fs.isOpened()) {
            ROS_ERROR("Failed to open file: %s", filename.c_str());
            exit(-1);
        }
    } catch(cv::Exception &ex) {
        ROS_ERROR("Parse error: %s", ex.what());
        exit(-1);
    }

    fs["odometry_topic"] >> this->odometry_topic;
    fs["source_frame"] >> this->source_frame;
    fs["target_frame"] >> this->target_frame;
    fs["output_file_dir"] >> this->output_file_dir;


    // Example on how to go deep in the tree Camera.Width
//    if (fs["Camera.Width"].isNamed()) fs["Camera.Width"] >> camera_params_.w;


    fs.release();
}

void OdometryEvaluationCreator::readConfigurationFromParameterServer() {
    ros::param::get("/odometry_topic", this->odometry_topic);
    ros::param::get("/source_frame", this->source_frame);
    ros::param::get("/target_frame", this->target_frame);
    ros::param::get("/output_file_dir", this->output_file_dir);

}
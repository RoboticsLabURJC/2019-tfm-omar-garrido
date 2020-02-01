//
// Created by omar on 1/2/20.
//

#include "OdometryEvaluationCreator.h"

OdometryEvaluationCreator::OdometryEvaluationCreator() {
    this->odometry_topic = "/difodo/odometry";
    this->source_frame = "openni_camera";
    this->target_frame = "world";
    this->output_file_path = ""; //TODO retrieve from arguments.
    this->is_first_time = true;

    time_t now = time(NULL);
    struct tm tstruct;
    char buf[40];
    tstruct = *localtime(&now);
    //format: day DD-MM-YYYY
    strftime(buf, sizeof(buf), "%d_%m_%Y-%H_%M_%S", &tstruct);
    std::string filename = std::string(buf) + "_odometry_evaluation_output.txt";
    output_file.open(filename);
    if (!output_file) {
        ROS_ERROR_STREAM("Output file couldn be created");
        exit(-1);
    }
}

OdometryEvaluationCreator::~OdometryEvaluationCreator() {
    if (output_file.is_open()) output_file.close();
}

void OdometryEvaluationCreator::loadConfiguration() {
    // Read configuration parameters
    //TODO

    // INITIALIZATION
    this->odometry_sub = this->n.subscribe(this->odometry_topic,
                                                     1000,
                                                     &OdometryEvaluationCreator::odometryCallback,
                                                     this);

    this->tfListener = new tf2_ros::TransformListener(this->tfBuffer);
}

void OdometryEvaluationCreator::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    ROS_INFO_STREAM("LLEGA ODOMETRIA");

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

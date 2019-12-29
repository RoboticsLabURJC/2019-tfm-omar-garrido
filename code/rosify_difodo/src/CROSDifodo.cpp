//
// Created by omar on 7/12/19.
//
#include "CROSDifodo.h"

// MACRO USED TO KNOW MORE INFORMATION ABOUT THE PROCESS TAKING PLACE IN THE TERMINAL.
#define DEBUG
//#undef DEBUG

CROSDifodo::CROSDifodo() : mrpt::vision::CDifodo() {
    // DEFAULT ATTRIBUTES VALUES
    input_depth_topic = "/camera/depth/image_rect_raw";
    output_odom_topic = "/difodo/odometry";

    rows_orig = 480;
    cols_orig = 640;

    /**
     * The number of times we want to downsample the original resolution.
     * downsample = 1 means that no downsample will be made.
     * Check the original description from definitions in CDifodo.h
     */
    downsample = 1;

    camera_fps = 30;
    objective_fps = 30;

    // Controls the number of messages per second we want to send.
    loop_rate = new ros::Rate(objective_fps);

    ctf_levels = 5;

    rows_ctf = rows_orig / downsample;
    cols_ctf = cols_orig / downsample;

    fovh_degrees = 74.0f; //58.6;
    fovv_degrees = 62.0f;; //45.6;

    // NOTE: I dont know why but if set to true, the algorithm doesnt work...
    fast_pyramid = false;

    // Control booleans
    is_new_frame_available = false;
    is_cancel = false;

    // Just to initialize it, the first measurement wont be accurate but the followings will be.
    last_execution_time = (double) cv::getTickCount();
}

void CROSDifodo::loadConfiguration() {
    /*********************VALUES THAT WILL BE RETRIEVED BY A .LAUNCH OR CONFIG FILE*********************/
    // This method returns false if the value couldn be found, but we already have the default values in the constructor
    // so no need to check for it
    ros::param::get("/input_depth_topic", input_depth_topic);
    ros::param::get("/output_odom_topic", output_odom_topic);

    ros::param::get("/rows_orig", rows_orig);
    ros::param::get("/cols_orig", cols_orig);

    int aux; // This is needed since get only works with integer (int32) references and not any other uint or uint_16...
    if (ros::param::get("/downsample", aux)) {
        downsample = aux;
    }

    ros::param::get("/camera_fps", camera_fps);
    ros::param::get("/objective_fps", objective_fps);

    // Controls the number of messages per second we want to send.
    loop_rate = new ros::Rate(objective_fps);

    if (ros::param::get("/ctf_levels", aux)) {
        ctf_levels = aux;
    }

    rows_ctf = rows_orig / downsample;
    cols_ctf = cols_orig / downsample;

    ros::param::get("/fovh_degrees", fovh_degrees);
    ros::param::get("/fovv_degrees", fovv_degrees);

    ros::param::get("/fast_pyramid", fast_pyramid);

    ROS_INFO_STREAM(std::endl <<
                         "---------------------------------------------------------" << std::endl <<
                         "             CONFIGURATION PARAMETERS LOADED" << std::endl <<
                         "---------------------------------------------------------" << std::endl <<
                         "input_depth_topic: " << input_depth_topic << std::endl <<
                         "output_odom_topic: " << output_odom_topic << std::endl <<
                         "rows_orig: " << rows_orig << std::endl <<
                         "cols_orig: " << cols_orig << std::endl <<
                         "downsample: " << downsample << std::endl <<
                         "camera_fps: " << camera_fps << std::endl <<
                         "objective_fps: " << objective_fps << std::endl <<
                         "ctf_levels " << ctf_levels << std::endl <<
                         "fovh_degrees: " << fovh_degrees << std::endl <<
                         "fovv_degrees: " << fovv_degrees << std::endl <<
                         "fast_pyramid: " << fast_pyramid << std::endl);

    /******************SET DIFODO ATTRIBUTES VALUES*****************/
    this->fovh = M_PI * fovh_degrees / 180.0;
    this->fovv = M_PI * fovv_degrees / 180.0;

    // Resolution of the depth image that the algorithm will work with. After the downsampling.
    this->m_width = cols_orig / this->downsample;
    this->m_height = rows_orig / this->downsample;

    // Resolution of the finest level of the pyramid. Should be equal or lower than the image working resolution.
    if (rows_orig < rows_ctf) {
        ROS_ERROR("The rows_ctf (%d) cannot be more than those of the working depth image rows_orig/downsample = %d.",
                  rows_ctf, this->m_height);
        // Set the maximum allowed by default
        this->rows = rows_orig / downsample;
    } else {
        this->rows = rows_ctf;
    }

    if (cols_orig < cols_ctf) {
        ROS_ERROR("The cols_ctf (%d) cannot be more than those of the working depth image cols_orig/downsample = %d.",
                  cols_ctf, this->m_width);
        // Set the maximum allowed by default
        this->cols = cols_orig / downsample;
    } else {
        this->cols = cols_ctf;
    }

    this->fps = camera_fps;

    /******************Resize Matrices and adjust parameters*****************/
    // *****Pyramid setup*****
    // Resize each matrix to have the same number of matrices than levels of pyramid
    const unsigned int pyr_levels =
            std::round(log((float) this->m_width / this->cols) / log(2.f)) + this->ctf_levels;
    this->depth.resize(pyr_levels);
    this->depth_old.resize(pyr_levels);
    this->depth_inter.resize(pyr_levels);
    this->depth_warped.resize(pyr_levels);
    this->xx.resize(pyr_levels);
    this->xx_inter.resize(pyr_levels);
    this->xx_old.resize(pyr_levels);
    this->xx_warped.resize(pyr_levels);
    this->yy.resize(pyr_levels);
    this->yy_inter.resize(pyr_levels);
    this->yy_old.resize(pyr_levels);
    this->yy_warped.resize(pyr_levels);
    this->transformations.resize(pyr_levels);

    // Resize each level (or matrix) of the pyramid to the desired (rows,cols) size
    for (unsigned int i = 0; i < pyr_levels; i++) {
        unsigned int s = pow(2.f, int(i));
        this->cols_i = this->m_width / s;
        this->rows_i = this->m_height / s;
        this->depth[i].resize(this->rows_i, this->cols_i);
        this->depth_inter[i].resize(this->rows_i, this->cols_i);
        this->depth_old[i].resize(this->rows_i, this->cols_i);
        this->depth[i].fill(0.0f);
        this->depth_old[i].fill(0.0f);
        this->xx[i].resize(this->rows_i, this->cols_i);
        this->xx_inter[i].resize(this->rows_i, this->cols_i);
        this->xx_old[i].resize(this->rows_i, this->cols_i);
        this->xx[i].fill(0.0f);
        this->xx_old[i].fill(0.0f);
        this->yy[i].resize(this->rows_i, this->cols_i);
        this->yy_inter[i].resize(rows_i, this->cols_i);
        this->yy_old[i].resize(this->rows_i, this->cols_i);
        this->yy[i].fill(0.0f);
        this->yy_old[i].fill(0.0f);
        this->transformations[i].resize(4, 4);

        if (this->cols_i <= this->cols) {
            this->depth_warped[i].resize(this->rows_i, this->cols_i);
            this->xx_warped[i].resize(this->rows_i, this->cols_i);
            this->yy_warped[i].resize(this->rows_i, this->cols_i);
        }
    }

    // Prepare the depth image for the resolution of the images after the downsample
    this->depth_wf.setSize(this->m_height, this->m_width);
}

void CROSDifodo::loadFrame() {
    // Check if resize is needed
    if (this->downsample != 1) {
        double resize_factor = 1.f / this->downsample;
        cv::resize(this->cv_copy_ptr->image, this->cv_copy_ptr->image, cv::Size(), resize_factor, resize_factor);
    }

    // Load in the depth image so the DIFODO algorithm can used it to process the algorithm
    cvBrigdeToMRPTMat(this->cv_copy_ptr, this->depth_wf);
}

void CROSDifodo::start() {
    // Initialize the image transport subscriber
    image_transport::ImageTransport it = image_transport::ImageTransport(nh);
    this->it_sub_depth = it.subscribe(input_depth_topic,
                                      1,
                                      &CROSDifodo::callbackImageDepthRaw,
                                      this);

    // Initialize the publisher
    this->odom_publisher = this->nh.advertise<nav_msgs::Odometry>(output_odom_topic, 30);

    // Initialize the main thread of the algorithm Difodo
    difodo_thread = std::thread(&CROSDifodo::run_difodo, this);
}

void CROSDifodo::cancel() {
    this->is_cancel = true;

    this->odom_publisher.shutdown();
    this->it_sub_depth.shutdown();

    // Wait for difodo thread to finish
    this->difodo_thread.join();
}

void CROSDifodo::callbackImageDepthRaw(const sensor_msgs::Image::ConstPtr &msg) {
#ifdef DEBUG
    if (this->is_new_frame_available) {
        ROS_WARN("Frame Dropped");
    }
#endif
    //NOTE: Is this mutex really neccesary? I think that it may not since if the new image gets replace it still is a
    // new image anyway...
    std::lock_guard<std::mutex> guard(this->frame_mutex);
    this->rosMsgToCvBridgePtr(msg, this->cv_copy_ptr);
    this->is_new_frame_available = true;
}

/**
 * Conversion from a ros depth image msg to a cv_bridge pointer that contains a cv::Mat. It creates a new copy of the
 * msg data since it may be needed to change the value if downsampling is applied and for the consumer to use it.
 * @param msg : The original ros message with the image information.
 * @param cv_copy_ptr : The pointer where the data will be copied.
 */
void CROSDifodo::rosMsgToCvBridgePtr(const sensor_msgs::Image::ConstPtr &msg, cv_bridge::CvImagePtr &cv_copy_ptr) {
    try {
        cv_copy_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

/**
 * There are probably better and FASTER ways to loop over a cv::Mat, see:
 * https://docs.opencv.org/2.4/doc/tutorials/core/how_to_scan_images/how_to_scan_images.html#howtoscanimagesopencv
 * https://answers.opencv.org/question/38/what-is-the-most-effective-way-to-access-cvmat-elements-in-a-loop/
 * @param cv_ptr
 */
void CROSDifodo::cvBrigdeToMRPTMat(const cv_bridge::CvImagePtr &cv_ptr, mrpt::math::CMatrixFloat &depth_image) {

    for (uint row = 0; row < rows; row++) {
        for (uint col = 0; col < cols; col++) {

            //Esto hace una copia seguramente, habra que mirar las formas mas eficientes y en cualquier caso evitar la
            // copia sino obtener un puntero a la posicion de memoria u algo asi.
            // NOTE: The depth image from realsense comes in milimeters with 4 digits. 3215 will be 3.215 meteres
            // DIFODO uses meters in double precision.
            depth_image(row, col) = 0.001f * cv_ptr->image.at<uint16_t>(row, col);

            // TODO: Crear una funcion de filtering o hacer aqui un filtrado de la imagen de profundidad en funcion de
            //  distancia minima y maxima (Setear esos valores a 0) Mirar como lo hace en los ejemplos de
            //  DifOdometry-camera y DifOdometr_Datasets
            //  Check limit values based on resolution of the depth images for the realsense. Ill create a realsense
            //  mode where if we are using the realsense D435 we know exactly which values to use, or maybe just add
            //  these tow limit values as configuration parameters. (Better to have a complex configuration file but
            //  well explain that having everythin on code.
            //  https://www.intel.com/content/dam/support/us/en/documents/emerging-technologies/intel-realsense-technology/Intel-RealSense-D400-Series-Datasheet.pdf
            //if (*pDepth < 4500.f)
            //depth_wf(yc, xc) = 0.001f * (*pDepth);
            //else
            //depth_wf(yc, xc) = 0.f;
        }
    }
}

void CROSDifodo::rosMsgToMRPTMat(const sensor_msgs::Image::ConstPtr &msg) {
    // TODO: The ideal will be to convert directly to the desired format of mrpt. Quicker than going to opencv
    //  first and the to mrpt
//    cv_bridge::CvImageConstPtr cv_ptr;
//    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);

    rosMsgToCvBridgePtr(msg, this->cv_copy_ptr);

    // Check if resize is needed
    if (this->downsample != 1) {
        double resize_factor = 1.f / this->downsample;
        cv::resize(cv_copy_ptr->image, cv_copy_ptr->image, cv::Size(), resize_factor, resize_factor);
    }

    // TODO: This ideally should return a MRPT mat but since i dont want to include mrpt includes if possible
    //  (only DIFODO) i just change the this->depth_wf parameter
    cvBrigdeToMRPTMat(this->cv_copy_ptr, this->depth_wf);

//        // Same information //2560
//        msg->step;
//        cv_ptr->image.step;
//
//
//        // Same information //16UC1
//        msg->encoding;
//        cv_ptr->encoding;
//
//        // Same information
//        msg->width;
//        msg->height;
//        cv_ptr->image.rows;
//        cv_ptr->image.cols;
}

geometry_msgs::TransformStamped CROSDifodo::createTransformStampedMsg(double pos_x, double pos_y, double pos_z,
                                                                      double roll, double pitch, double yaw,
                                                                      ros::Time current_time) {
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = pos_x;
    odom_trans.transform.translation.y = pos_y;
    odom_trans.transform.translation.z = pos_z;
    odom_trans.transform.rotation = odom_quat;

    return odom_trans;
}

nav_msgs::Odometry CROSDifodo::createOdometryMsg(double pos_x, double pos_y, double pos_z,
                                                 double roll, double pitch, double yaw, ros::Time current_time) {
    /*
     * "odom" is the conventional name for the central or absolute coordinate system. We could say is the
     * world coordinate system and is often used as the reference system for several robots.
     */
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
//    odom.header.seq // This will be filled by default
//    odom.child_frame_id

    // Position
    odom.pose.pose.position.x = pos_x;
    odom.pose.pose.position.y = pos_y;
    odom.pose.pose.position.z = pos_z;

    // First pass from euler angles to quaternion.
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(roll, pitch, yaw);
    // Orientation
    odom.pose.pose.orientation.x = quat_tf.x();
    odom.pose.pose.orientation.y = quat_tf.y();
    odom.pose.pose.orientation.z = quat_tf.z();
    odom.pose.pose.orientation.w = quat_tf.w();

    //Covariance explained: http://manialabs.wordpress.com/2012/08/06/covariance-matrices-with-a-practical-example/
    // Both matrices are 6*6. array of double [36] for odometry.covariance while the getcovaraince() is [6, 6]
    int i = 0;
    for(auto iter=this->getCovariance().cbegin(); iter != this->getCovariance().cend(); iter++) {
        odom.pose.covariance[i] = static_cast<double>(*iter);
        i++;
    }

    // Set velocity
    /* Velocity is often used to get the next position of the robot instead of getting the pose and then computing
     * the velocity. In this case we compute the speed or velocity based on the previous position and the new
     * position of our robot, in this case a camera.
     *  "base_link" is the conventional name used for the "robot" based coordinate system. So the velocity is related
     *  or referred to the robot itself which is the coordinate system "base_link". We could say that base link is
     *  moving (due to this speed) in reference to the central coordinate system called "odom".
     */
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = kai_loc.vx;
    odom.twist.twist.linear.y = kai_loc.vy;
    odom.twist.twist.linear.z = kai_loc.vz;
    odom.twist.twist.angular.x = kai_loc.wx;
    odom.twist.twist.angular.y = kai_loc.wy;
    odom.twist.twist.angular.z = kai_loc.wz;

    //    odom.twist.covariance

    return odom;
}

void CROSDifodo::publishOdometryMsgs(double pos_x, double pos_y, double pos_z,
                                     double roll, double pitch, double yaw) {
    // In order to be correctly visualize by RVIZ, I have to change "y" and "z" translations and rotations sign
    pos_y = -pos_y;
    pos_z = -pos_z;
    pitch = -pitch;
    yaw = -yaw;

    ros::Time current_time = ros::Time::now();

    // ********************PUBLISHING THE TRANSFORMATION BETWEEN FRAMES********************
    geometry_msgs::TransformStamped odom_trans =
            CROSDifodo::createTransformStampedMsg(pos_x, pos_y, pos_z, roll, pitch, yaw, current_time);

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    // ********************PUBLISHING THE ODOMETRY MESSAGE IN ROS********************
    nav_msgs::Odometry odom_msg = createOdometryMsg(pos_x, pos_y, pos_z, roll, pitch, yaw, current_time);
    //Publish the message
    this->odom_publisher.publish(odom_msg);
}

void CROSDifodo::execute_iteration() {
    {
        //unlocks the mutex when goes out of scope with the destructor.
        std::lock_guard<std::mutex> guard(this->frame_mutex);
        if (this->is_new_frame_available) {
            this->loadFrame();
            this->is_new_frame_available = false;
        } else {
            // Return, we will sleep the necessary time with ros::rate sleep method when returning.
            return;
        }
    }

    // Compute and publish odometry
    this->odometryCalculation();

    //Publish pose
    //ROS_INFO_STREAM("New pose" << this->cam_pose);

    this->publishOdometryMsgs(this->cam_pose.x(), this->cam_pose.y(), this->cam_pose.z(),
                              this->cam_pose.roll(), this->cam_pose.pitch(), this->cam_pose.yaw());

    // Compute execution times
#ifdef DEBUG
    // Compute the working time or working frame rate
    double working_time = ((double) cv::getTickCount() - this->last_execution_time) / cv::getTickFrequency() * 1000;
    this->last_execution_time = (double) cv::getTickCount();
    float working_fps = 1000 / working_time;

    // Add an offset in the FPS of 0.5 so if is working at 29.98 instead of 30 FPS we dont raise any warning
    if (working_fps + 0.5 < this->objective_fps) {
        ROS_WARN_STREAM("The algorithm cannot handle objective_fps configured, "
                        "running at (" << working_fps << ") FPS");
    }

    if (this->execution_time > (1000 / this->objective_fps)) {
        ROS_WARN_STREAM("The algorithm takes more time to process an image tham the objective_fps configured"
        );
    }

    ROS_INFO_STREAM("Execution time of DifOdo only " << this->execution_time << " ms");
    ROS_INFO_STREAM("Executing at:                 " << working_time << " ms");
    ROS_INFO_STREAM("Working at FPS:               " << working_fps << " FPS");
#endif
}

void CROSDifodo::run_difodo() {
    loop_rate->reset();  // Start the timer start to measure correctly from this time to the next sleep
    while (!this->is_cancel) {
        this->execute_iteration();
        // WARNING: For whatever the reason when using this sleep the algorithm time "execution_time" increase from 8ms
        // to 12ms... This makes no sense since execution time is the internal execution time of the method
        // odometryCalculation() from Difodo...So
        // But when working with lower objective_fps like 1-5-10 Hz is useful to publish at a constant frequency.
        this->loop_rate->sleep();
    }
}
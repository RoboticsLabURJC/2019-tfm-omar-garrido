//
// Created by omar on 7/12/19.
//
#include "CROSDifodo.h"

// MACRO USED TO KNOW MORE INFORMATION ABOUT THE PROCESS TAKING PLACE IN THE TERMINAL.
#define DEBUG
//#undef DEBUG

CROSDifodo::CROSDifodo() : mrpt::vision::CDifodo() {
    // DEFAULT ATTRIBUTES VALUES
    first_iteration = true;

    input_depth_topic = "/camera/depth/image_rect_raw";
    output_odom_topic = "/difodo/odometry";

    rows_orig = 480;
    cols_orig = 640;

    depth_pixel_scale = 1000;

    min_depth_value_filter = 0.5f;
    max_depth_value_filter = 4.5f;

    /**
     * The number of times we want to downsample the original resolution.
     * downsample = 1 means that no downsample will be made.
     * Check the original description from definitions in CDifodo.h
     */
    downsample = 1;

    objective_fps = 30;

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

    log_pose_relative_to_world = false;

    time_t now = time(NULL);
    struct tm tstruct;
    char buf[40];
    tstruct = *localtime(&now);
    //format: day DD-MM-YYYY
    strftime(buf, sizeof(buf), "%d_%m_%Y-%H_%M_%S", &tstruct);
    std::string filename = std::string(buf) + "_rosify_difodo_odom_output.txt";
    output_file.open(filename);
    if (!output_file) {
        ROS_ERROR_STREAM("Output file couldn be created");
        exit(-1);
    }
}

CROSDifodo::~CROSDifodo() {
    if (output_file.is_open()) output_file.close();
}

void CROSDifodo::loadConfiguration() {
    /*********************VALUES THAT WILL BE RETRIEVED BY A .LAUNCH OR CONFIG FILE*********************/
    // This method returns false if the value couldn be found, but we already have the default values in the constructor
    // so no need to check for it
    ros::param::get("/input_depth_topic", input_depth_topic);
    ros::param::get("/output_odom_topic", output_odom_topic);

    ros::param::get("/rows_orig", rows_orig);
    ros::param::get("/cols_orig", cols_orig);

    ros::param::get("/depth_pixel_scale", depth_pixel_scale);

    ros::param::get("/min_depth_value_filter", min_depth_value_filter);
    ros::param::get("/max_depth_value_filter", max_depth_value_filter);

    int aux; // This is needed since get only works with integer (int32) references and not any other uint or uint_16...
    if (ros::param::get("/downsample", aux)) {
        downsample = aux;
    }

    ros::param::get("/objective_fps", objective_fps);

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

void CROSDifodo::loadInnerConfiguration() {
    // This method returns false if the value couldn be found, but we already have the default values in the constructor
    // so no need to check for it
    bool IS_TUM_DATASET = true;

    if (IS_TUM_DATASET) {
        // FOR TUM use this
        log_pose_relative_to_world = true;
        world_frame_name = "world";
        odom_frame_name = "openni_camera";

        input_depth_topic = "/camera/depth/image";
        output_odom_topic = "/difodo/odometry";

        // Realsense D435 = 74.0f ; TUM dataset = 58.5;
        // Realsense D435 = 62.0f ; TUM dataset = 46.6;
        fovh_degrees = 58.5f;
        fovv_degrees = 46.6f;

        // Realsense D435 = 1000 ; TUM dataset = 1 (5000 in their website and also 16bit) for the rosbag it seems 32bit and
        // scale of 1
        depth_pixel_scale = 1;

    } else { // REALSENSE D435
        log_pose_relative_to_world = false;

        input_depth_topic = "/camera/depth/image_rect_raw";
        output_odom_topic = "/difodo/odometry";

        // Realsense D435 = 74.0f ; TUM dataset = 58.5;
        // Realsense D435 = 62.0f ; TUM dataset = 46.6;
        fovh_degrees = 74.0f;
        fovv_degrees = 62.0f;

        // Realsense D435 = 1000 ; TUM dataset = 1 (5000 in their website and also 16bit) for the rosbag it seems 32bit and
        // scale of 1
        depth_pixel_scale = 1000;
    }

    rows_orig = 480;
    cols_orig = 640;

    min_depth_value_filter = 0.1f;
    max_depth_value_filter = 7.0f;

    /**
     * The number of times we want to downsample the original resolution.
     * downsample = 1 means that no downsample will be made.
     * Check the original description from definitions in CDifodo.h
     */
    downsample = 4;

    objective_fps = 0;

    ctf_levels = 5;

    rows_ctf = rows_orig / downsample;
    cols_ctf = cols_orig / downsample;

    // NOTE: I dont know why but if set to true, the algorithm doesnt work...
    fast_pyramid = false;

    // Control booleans
    is_new_frame_available = false;
    is_cancel = false;

    // Just to initialize it, the first measurement wont be accurate but the followings will be.
    last_execution_time = (double) cv::getTickCount();

    ROS_INFO_STREAM(std::endl <<
                              "---------------------------------------------------------" << std::endl <<
                              "             CONFIGURATION PARAMETERS LOADED" << std::endl <<
                              "---------------------------------------------------------" << std::endl <<
                              "input_depth_topic: " << input_depth_topic << std::endl <<
                              "output_odom_topic: " << output_odom_topic << std::endl <<
                              "rows_orig: " << rows_orig << std::endl <<
                              "cols_orig: " << cols_orig << std::endl <<
                              "downsample: " << downsample << std::endl <<
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
    double t0 = (double) cv::getTickCount(); //OGM
    if (this->downsample != 1) {
        double resize_factor = 1.f / this->downsample;
        cv::resize(this->cv_copy_ptr->image, this->cv_copy_ptr->image, cv::Size(), resize_factor, resize_factor);
    }

    // Load in the depth image so the DIFODO algorithm can used it to process the algorithm
    cvBrigdeToMRPTMat(this->cv_copy_ptr, this->depth_wf);
    double time_waste = ((double) cv::getTickCount() - t0) / cv::getTickFrequency() * 1000; //OGM
    ROS_INFO_STREAM("OGM::TIME READING FRAME:: " << time_waste << "ms");
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

    // Initialize the tf listener
    // (Note: This needs some time before actually getting the tf, in order to fill the buffer)
    this->tfListener = new tf2_ros::TransformListener(this->tfBuffer);
}

void CROSDifodo::cancel() {
    this->is_cancel = true;

    this->odom_publisher.shutdown();
    this->it_sub_depth.shutdown();

    // Wait for difodo thread to finish
    this->difodo_thread.join();
}

void CROSDifodo::callbackImageDepthRaw(const sensor_msgs::Image::ConstPtr &msg) {
    ROS_INFO("OGM: New frame received");
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
        if (first_iteration) {
            // Do some verifications against the dimensions expected of the image and the one that has been received from
            // ros messages
            if (msg->height != this->rows_orig) {
                ROS_ERROR("The rows_orig(%d) specified is not equal to the image height(%d)", this->rows_orig,
                          msg->height);
                exit(-1);
            }
            if (msg->width != this->cols_orig) {
                ROS_ERROR("The cols_orig(%d) specified is not equal to the image width(%d)", this->cols_orig,
                          msg->width);
                exit(-1);
            }

            // Check the encoding type 16bit or 32bit
            if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
                ROS_INFO("Encoding of the depth image is 16bit");
            } else if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
                ROS_INFO("Encoding of the depth image is 32bit");
            } else {
                ROS_ERROR_STREAM("Encoding neither 16bit nor 32bit. Exiting due to an unknown encoding type: " << msg->encoding);
            }
            // Keep the encoding for latter.
            this->pixel_encoding = msg->encoding;
        }
        // Keep the message temporal mark.
        this->current_depth_image_time = msg->header.stamp;

        cv_copy_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

/**
 * There are probably better and FASTER ways to loop over a cv::Mat, see:
 * https://docs.opencv.org/2.4/doc/tutorials/core/how_to_scan_images/how_to_scan_images.html#howtoscanimagesopencv
 * https://answers.opencv.org/question/38/what-is-the-most-effective-way-to-access-cvmat-elements-in-a-loop/
 * https://kezunlin.me/post/61d55ab4/#toc-heading-17 (Best, updated with foreach for new c++ 11)
 * @param cv_ptr
 */
void CROSDifodo::cvBrigdeToMRPTMat(const cv_bridge::CvImagePtr &cv_ptr, mrpt::math::CMatrixFloat &depth_image) {
    for (uint row = 0; row < rows; row++) {
        for (uint col = 0; col < cols; col++) {
            //Esto hace una copia seguramente, habra que mirar las formas mas eficientes y en cualquier caso evitar la
            // copia sino obtener un puntero a la posicion de memoria u algo asi.
            // NOTE: The depth image from realsense comes in milimeters with 4 digits. 3215 will be 3.215 meteres
            // DIFODO uses meters in double precision.
            double depth_pixel_value = 0.0f;

            if (this->pixel_encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
                depth_pixel_value = (1.0f / depth_pixel_scale) * cv_ptr->image.at<uint16_t >(row, col);
            } else if (this->pixel_encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
                depth_pixel_value = (1.0f / depth_pixel_scale) * cv_ptr->image.at<float>(row, col);
            }

            // FILTER DEPTH VALUES
            if(min_depth_value_filter <= depth_pixel_value && depth_pixel_value <= max_depth_value_filter){  // Represents expected pre-REP logic and is the only necessary condition for most applications.
                // This is a valid measurement.
                depth_image(row, col) = depth_pixel_value;
            } else if(!isfinite(depth_pixel_value) && depth_pixel_value < 0){
                // Object too close to measure.
                depth_image(row, col) = 0.0f;
            } else if(!isfinite(depth_pixel_value) && depth_pixel_value > 0){
                // No objects detected in range.
                depth_image(row, col) = 0.0f;
            } else if( isnan(depth_pixel_value) ){
                // This is an erroneous, invalid, or missing measurement.
                depth_image(row, col) = 0.0f;
            } else {
                // The sensor reported these measurements as valid, but they are discarded per the limits defined by minimum_range and maximum_range.
                depth_image(row, col) = 0.0f;
            }

            // TODO: Crear una funcion de filtering o hacer aqui un filtrado de la imagen de profundidad en funcion de
            //  distancia minima y maxima (Setear esos valores a 0) Mirar como lo hace en los ejemplos de
            //  DifOdometry-camera y DifOdometr_Datasets
            //  Check limit values based on resolution of the depth images for the realsense. Ill create a realsense
            //  mode where if we are using the realsense D435 we know exactly which values to use, or maybe just add
            //  these tow limit values as configuration parameters. (Better to have a complex configuration file but
            //  well explain that having everythin on code.
            //  https://www.intel.com/content/dam/support/us/en/documents/emerging-technologies/intel-realsense-technology/Intel-RealSense-D400-Series-Datasheet.pdf
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

geometry_msgs::Pose CROSDifodo::fromMrptPoseToROSGeometryPose(double pos_x, double pos_y, double pos_z,
                                                              double roll, double pitch, double yaw) {
    geometry_msgs::Pose  pose;

    pose.position.x = pos_x;
    pose.position.y = pos_y;
    pose.position.z = pos_z;

    // First pass from euler angles to quaternion.
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(roll, pitch, yaw);
    // Orientation
    pose.orientation.x = quat_tf.x();
    pose.orientation.y = quat_tf.y();
    pose.orientation.z = quat_tf.z();
    pose.orientation.w = quat_tf.w();

    return pose;
}

geometry_msgs::TransformStamped CROSDifodo::createTransformStampedMsg(geometry_msgs::Pose pose, ros::Time current_time) {
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = pose.position.x;
    odom_trans.transform.translation.y = pose.position.y;
    odom_trans.transform.translation.z = pose.position.z;
    odom_trans.transform.rotation.x = pose.orientation.x;
    odom_trans.transform.rotation.y = pose.orientation.y;
    odom_trans.transform.rotation.z = pose.orientation.z;
    odom_trans.transform.rotation.w = pose.orientation.w;

    return odom_trans;
}

nav_msgs::Odometry CROSDifodo::createOdometryMsg(geometry_msgs::Pose pose, ros::Time current_time) {
    /*
     * "odom" is the conventional name for the fixed frame where the robot started.
     */
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
//    odom.header.seq // This will be filled by default

    odom.pose.pose = pose;

    //Covariance explained: http://manialabs.wordpress.com/2012/08/06/covariance-matrices-with-a-practical-example/
    // Both matrices are 6*6. array of double [36] for odometry.covariance while the getcovaraince() is [6, 6]
    auto double_covariance_mtx = this->getCovariance().cast_double();
    int elements_count = this->getCovariance().rows() * this->getCovariance().cols();
    for(int i = 0;  i < elements_count; i++) {
        odom.pose.covariance[i] = double_covariance_mtx(i);
    }

    // Set velocity
    /* Velocity is often used to get the next position of the robot instead of getting the pose and then computing
     * the velocity. In this case we compute the speed or velocity based on the previous position and the new
     * position of our robot, in this case a camera.
     *  "base_link" is the conventional name used for the "robot" based coordinate system. So the velocity is related
     *  or referred to the robot itself which is the coordinate system "base_link". We could say that base link is
     *  moving (due to this speed) in reference to the central coordinate system called "odom".
     */
    odom.twist.twist.linear.x = kai_loc.vx;
    odom.twist.twist.linear.y = kai_loc.vy;
    odom.twist.twist.linear.z = kai_loc.vz;
    odom.twist.twist.angular.x = kai_loc.wx;
    odom.twist.twist.angular.y = kai_loc.wy;
    odom.twist.twist.angular.z = kai_loc.wz;
    //    odom.twist.covariance

    return odom;
}

void CROSDifodo::publishOdometryMsgs(geometry_msgs::Pose pose) {
    ros::Time current_time = ros::Time::now();

    // ********************PUBLISHING THE TRANSFORMATION BETWEEN FRAMES********************
    geometry_msgs::TransformStamped odom_trans =
            CROSDifodo::createTransformStampedMsg(pose, current_time);

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    // ********************PUBLISHING THE ODOMETRY MESSAGE IN ROS********************
    nav_msgs::Odometry odom_msg = createOdometryMsg(pose, current_time);
    //Publish the message
    this->odom_publisher.publish(odom_msg);
}

double CROSDifodo::controlWorkingRate() {
    double working_time_ms = 0;
    double time_sleeping = (double) cv::getTickCount();

    if (!first_iteration) {
        working_time_ms = ((double) cv::getTickCount() - this->last_execution_time) / cv::getTickFrequency() * 1000;
        if (objective_fps != 0) {
            ROS_INFO_STREAM("Time to sleep: " << 1000.0f / objective_fps - working_time_ms << " ms");
            double usec_to_sleep = (1000.0f / objective_fps - working_time_ms) * 1000;
            if (usec_to_sleep > 0) {
                usleep(usec_to_sleep); //Its un micro seconds
            }
        }
    }
    time_sleeping = ((double) cv::getTickCount() - time_sleeping) / cv::getTickFrequency() * 1000;
    this->last_execution_time = (double) cv::getTickCount();
    return working_time_ms + time_sleeping;
}

void CROSDifodo::logTimesOfExecution(double working_time_ms) {
    // Compute the working time or working frame rate
    float working_fps = 1000 / working_time_ms;

    if (objective_fps != 0) {
        // Add an offset in the FPS of 0.5 so if is working at 29.98 instead of 30 FPS we dont raise any warning
        if (working_fps + 0.5 < this->objective_fps) {
            ROS_WARN_STREAM("The algorithm cannot handle objective_fps configured, "
                            "running at (" << working_fps << ") FPS");
        }

        if (this->execution_time > (1000 / this->objective_fps)) {
            ROS_WARN_STREAM("The algorithm takes more time to process an image than the objective_fps configured"
            );
        }
    }

    ROS_INFO_STREAM("Execution time of DifOdo only " << this->execution_time << " ms");
    ROS_INFO_STREAM("Executing at:                 " << working_time_ms << " ms");
    ROS_INFO_STREAM("Working at FPS:               " << working_fps << " FPS");
    //Print Pose
    ROS_INFO_STREAM("New pose" << this->cam_pose);
}

void CROSDifodo::getTransform() {
    // Acquire the current position and orientation of the desired frame, in order to use it to transform the odometry
    // from DIFODO to the "world"/"map" frame.
    if (first_iteration) {
        try {
            // We get the transformation between frames. (world to odom frames)
            transformStamped = tfBuffer.lookupTransform(this->world_frame_name, this->odom_frame_name,
                                                        ros::Time(0), ros::Duration(0,100000));
            ROS_INFO_STREAM(transformStamped);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ROS_ERROR("One of the frames specified seem not to be found on the /tf topic. If you dont want to apply any"
                      " tranformation to generate the log file against the 'world' frame instead of 'odom' then set "
                      "'log_pose_relative_to_world' configuration parameter to false");
            exit(-1);
        }
    }
}

void CROSDifodo::writePoseToFile(double tx, double ty, double tz, double qx,  double qy, double qz,  double qw) {
    if (output_file.is_open())
    {
        //https://www.quora.com/How-do-I-customize-cout-in-C++-to-print-n-digit-after-the-decimal-point-of-a-floating-point
        output_file << std::fixed <<  std::setprecision(4) \
                        << current_depth_image_time.toSec() << " " \
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

void CROSDifodo::executeIteration() {
    // 1. Load frame if a new frame is available otherwise return and retry in the next iteration
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

    double t0 = (double) cv::getTickCount(); //OGM

    // 1.5 Set the frame rate between the previous frame and the current frame. This is a value used in the
    // odometryCalcuation of DIFODO algorithm, when computing temporal derivatives. Since the frame rate can vary (for
    // example if we dropped frames) it has to be recalculated on each iteration and set before computing the odometry.
    if (!first_iteration) {
        ros::Duration time_between_images = this->current_depth_image_time - this->last_depth_image_time;
        this->fps = 1.0f / time_between_images.toSec();
    }
    this->last_depth_image_time = this->current_depth_image_time;

    // 2. Compute and publish odometry
    this->odometryCalculation();

    // 2.5 The pose estimated from DIFODO seem to be with the Y and Z axis inverted. We inverted them here so the pose is
    // the correct one.
    // In order to be correctly visualize by RVIZ, I have to change "y" and "z" translations and rotations sign
    geometry_msgs::Pose pose_inverted = fromMrptPoseToROSGeometryPose(this->cam_pose.x(), -this->cam_pose.y(), -this->cam_pose.z(),
                                                    this->cam_pose.roll(), -this->cam_pose.pitch(), -this->cam_pose.yaw());

    // 3. Control the working rate: Wait the time needed to publish at the constant rate specified
    // Compute the working time or working frame rate.
    double working_time_ms = this->controlWorkingRate();

#ifdef DEBUG
    this->logTimesOfExecution(working_time_ms);
#endif

    // 4. Publish the odometry messages and transforms.
    this->publishOdometryMsgs(pose_inverted);

    // 5. Get if there is one, the transformation from "map" or "world" to the "odom" frame.
    if (log_pose_relative_to_world) {
        if (first_iteration) {
            this->getTransform();
        } else {
            //https://answers.ros.org/question/273205/transfer-a-pointxyz-between-frames/
            geometry_msgs::Pose pose_transformed;

            //TODO: Check that there is transformStamped (null or something)
            //Apply transformation to pose_stamped_in and get the pose transform on pose_transformed
            tf2::doTransform(pose_inverted, pose_transformed, transformStamped);

            // 6. Write the output to the output file.
            this->writePoseToFile(pose_transformed.position.x, pose_transformed.position.y, pose_transformed.position.z,
                                  pose_transformed.orientation.x, pose_transformed.orientation.y, pose_transformed.orientation.z,
                                  pose_transformed.orientation.w);
        }
    } else {
        this->writePoseToFile(pose_inverted.position.x, pose_inverted.position.y, pose_inverted.position.z,
                              pose_inverted.orientation.x, pose_inverted.orientation.y, pose_inverted.orientation.z,
                              pose_inverted.orientation.w);
    }

    if (first_iteration) first_iteration = false;

    double time_waste = ((double) cv::getTickCount() - t0) / cv::getTickFrequency() * 1000; //OGM
    ROS_INFO_STREAM("OGM::TIME:: " << time_waste << "ms");
}

void CROSDifodo::run_difodo() {
    while (!this->is_cancel) {
        this->executeIteration();
    }
}

---
permalink: /install/

title: "Installation and use"

sidebar:
  nav: "docs"

toc: true
toc_label: "Index"
toc_icon: "cog"
classes: wide


---
# 1. Rosify Difodo
Check the instrucctions and get the code from [https://github.com/RoboticsLabURJC/2019-tfm-omar-garrido/tree/master/code/rosify_difodo](https://github.com/RoboticsLabURJC/2019-tfm-omar-garrido/tree/master/code/rosify_difodo)

## Requisites
The latest version of **Rosify Difodo** has been tested and is currently working on Ubuntu 18.04 LTS but it should work on other platforms. It has been tested with latest [ros-melodic](http://wiki.ros.org/melodic).
**(NOT WORKING on Ubuntu 16.04)** It should work with other ros versions like **ros-kinetics and Ubuntu 16.04** but havent been tested yet.

**Note: ros-melodic only supports officially Ubuntu 18.04 while ros-kinetics supports only Ubuntu 16.04**

### CMake
In order to compile and install the vast majority of C++ based softwares CMake is widely used.
Everything has been tested with **version 3.10.2**.
Also lower version will work we recommend to have the latest cmake version for your OS since a lot of packages depend on it to be installed.

### OpenCV
When installing ROS also an OpenCV version is installed in the system. We recommend to use that since it wont give you any headache. But you can always build your OpenCV version and the build ROS from sources.
Ros-melodic comes with OpenCV 3.2.0

### ROS
- If your are using Ubuntu 18.04 install latest ros-melodic following the [official instructions](http://wiki.ros.org/melodic/Installation/Ubuntu).
- **(NOT WORKING)** If your are using Ubuntu 16.04 is better to install ros-kinetics from [official instructions](http://wiki.ros.org/kinetic/Installation/Ubuntu).

**Also you can always try to compile ros from sources in your desired Ubuntu version we do not recommend it.**


### MRPT
Difodo algorithm is implemented in the MRPT platform so the package Rosify Difodo depends on their library.
Difodo wasnt fully functionaly when I started testing it but since [this commit](https://github.com/MRPT/mrpt/commit/31e853f504294c1b1ed706b2c3c525c945d63dc0) is working. You can download the exact sources from that commit [here](https://github.com/MRPT/mrpt/tree/31e853f504294c1b1ed706b2c3c525c945d63dc0), this way you can be sure that you are using a version of the MRPT platform where DIFODO was working, although you will have to compile the mrpt lib by yourself.
[Compilet MRPT from sources](https://github.com/MRPT/mrpt#32-build-from-sources).

**Recommended:** Before going crazy and compile from sources I would first try to install within their ppa repositories because probably those versions will work.
Follow the instructions from https://github.com/MRPT/mrpt#3-install.
Stable Version 1.5 probably wont work since I tested before and since is a stable version is likely to not get updated so install version 1.9 also called 2.X using their [private ppa](https://github.com/MRPT/mrpt/tree/31e853f504294c1b1ed706b2c3c525c945d63dc0#31-ubuntu):
```
sudo add-apt-repository ppa:joseluisblancoc/mrpt
sudo apt-get update
sudo apt-get install libmrpt-dev mrpt-apps
```
The version in this repository gets updated very frequently since is a development branch. 
The last snapshot or version that I worked with is **1:1.9.9~snapshot20191122-1444-git-898be015-bionic-1** but I encouraged you to work with their latest.

**Test and problems:** See this [entry from my blog](https://roboticslaburjc.github.io/2019-tfm-omar-garrido/entries/entry7/) to see how to check MPRT with Difodometry-Dataset app and also in case you have a problem with the installation if may have an answer here.

### (Optional requirements for realsense cameras use)
If you would like to use a realsense camera with ros then you will have to install the following:

#### (Optional) Realsense SDK 2.0
Follow the instructions from [https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)
#### (Optional) Ros-realsense
Follow official installation instructions from https://github.com/IntelRealSense/realsense-ros.
NOTE: If you plan to use **ros-melodic** at the time of writing there is no official ubuntu package to install ros-realsense package and it seems there wont be any soon. So you will have to install it from sources. In my blog I already did this, the steps can be follow on [Week 12-13 Realsense D435 on Ubuntu 18.04](https://roboticslaburjc.github.io/2019-tfm-omar-garrido/entries/entry5/#1-realsense-d435-ubuntu-1804)

#### (Optional) rs_rgbd package
Some examples to use realsense camera are done with **rs_rgb.launch** which is not installed by default but can be simply installed with:

**sudo apt install ros-melodic-rgbd-launch**
or
**sudo apt install ros-kinetic-rgbd-launch**


## Install as a ros package
**First get the code from github:**
```
git clone https://github.com/RoboticsLabURJC/2019-tfm-omar-garrido.git
```

**Move or copy the package to a catking workspace or your ros workspace**
In my case:
```
cp -r rosify_difodo/ ~/Programs/catkin_ws/src/
```

If you dont know where your catkin workspace is or you havent created one:
- See the [section](https://roboticslaburjc.github.io/2019-tfm-omar-garrido/entries/entry9/#creating-a-launch-file).
- [Create a catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

**To see if the package has been correctly install and is being recognized:**
```
rospack list | grep -i difodo
```
This command should return the **rosify\_difodo** package and its location.

**Build the application, generate the binary node**
```
# Go to your catkin_ws root.
cd  ~/Programs/catkin_ws
# Build
catkin_make
```

The command **catkin_make** will build the packages within your catkin workspace. In order to just generate the desired package:
```
# Once this in done, from now on only this package will be build even with catkin_make
catkin_make --only-pkg-with-deps rosify_difodo
# Be sure to do this to be able to build again all the packages
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
```

To see other options to build packages and manage catkin see this [answer](https://answers.ros.org/question/54178/how-to-build-just-one-package-using-catkin_make/) that also uses:

```
sudo apt-get install python-catkin-tools
```

**All the code above in just one code block for copy paste:**
```
git clone https://github.com/RoboticsLabURJC/2019-tfm-omar-garrido.git
cp -r rosify_difodo/ ~/Programs/catkin_ws/src/
cd  ~/Programs/catkin_ws
catkin_make --only-pkg-with-deps rosify_difodo
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
```

## Install with cmake
In case you want to build the software without being a ros package and just create a C++ binary executable you can build as with any other piece of software using cmake.

```
cd rosify_difodo/
mkdir build
cd build/

# Build with CMake
cmake ..
# or to build a release version type instead
cmake .. -DCMAKE_BUILD_TYPE=Release

# CREATE THE EXECUTABLE
make
# or to use 4 process in case you have a CPU with at least 4 process ype:
make -j4

# The executable will have been created in:
cd devel/lib/rosify_difodo/
ls
```

## Usage
**These are the instructions to make the algorithm work with realsense D435**. Since this is a ros package it uses .launch configuration files to configure the DIFODO algorithm with the camera settings like FOV and the resolution that the depth images comes with, among others parameters.

**In case you want to use another camera you will have to modify the ros\_difodo.launch to correctly change the FOV, resolution, FPS and the depth topic for the camera that you will be using**

### Realsense camera (D435)
First we have to make the desire camera publish depth frames in the specified topic in the **ros_difodo.launch**
In my case I use realsense D435 (See previous posts).Then we run the package with the **ros_difodo.launch**.
```
# Run realsense camera with:
roslaunch realsense2_camera rs_rgbd.launch
# On another termianl Run difodo with:
roslaunch rosify_difodo **ros_difodo.launch**
```

**Ive created another set of launch files that runs the realsense node and the difodo algorithm all at once**

```
roslaunch rosify_difodo ros_difodo_and_realsense.launch
```
In order to change any configuration parameter of Difodo, the parameters are in the **ros_difodo.launch** since **ros_difodo_and_realsense.launch** loads the node from that launch file.

Also the realsense node has been integrated so if you want to change any parameter just change the **rs_rgbd.launch** from rosify_difodo package. Since **rs_rgbd.launch** includes others launch and xmls sometimes changes are overwritten by the new ones.

**Changing frequency of depth**
Realsense can operate on different FPS for different resolutions use command **rs-enumerate-devices** to see available resolutions and FPS.

The default resolution is 30FPS, to change the resolution that the camera send information you will have to change ```the launch file or overwrite like this the desired value:

```
roslaunch realsense2_camera rs_rgbd.launch depth_fps:=60
```

Since the FPS of the camera is an unknown parameter for DIFODO it also has to be in difodo configuration file or directly in the command line (this way the change is only temporarily applied into this execution)
```
roslaunch rosify_difodo ros_difodo.launch camera_fps:=60

# or everything in the same call
roslaunch rosify_difodo ros_difodo_and_realsense.launch.launch camera_fps:=60 depth_fps:=60
```

**For whatever the reason even though the changes are reflected in the execution, I cant get past the 30FPS for the depth frame, maybe is a hardware issue...**

Another interesting parameter to change is **working\_fps** in the ros_difodo.launch.
This parameter controls the maximum speed at what difodo works. (If hardware doesnt allow to work at that speed it will simply work at the maximum otherwise it will work at the specified rate and publish messages at that frequency)


### TUM dataset plus odometry_evaluation_file_creator

[Fully explain here.](https://roboticslaburjc.github.io/2019-tfm-omar-garrido/entries/entry14/#rosify_difodo-1) If what you would like to do is test this algorithm against a sequence of TUM RGBD dataset, do this and then play the rosbag sequence of the dataset:
```
roslaunch rosify_difodo ros_difodo_TUM_evaluation_file.launch
```

With this you will be running rosify_difodo and odometry_evaluation_file_creator, both configured to work together and with TUM dataset, creating a groundtruth that can be used for comparison later.


## Launch files and parameters
There are a few launch files which also set some parameters, here is a brief summary of them:

### ros\_difodo.launch
This node is primarily used to set up the parameters from the difodo algorithm. Without it the default values will be loaded.

- **input_depth_topic**:The topic where the depth images come from
- **input_depth_topic**:The topic where the odometry message is publish to. Default: /difodo/odometry
- **rows_orig**: rows or height of the depth image
- **cols_orig**: cols or width of the depth image
- **depth_pixel_scale**: The scale of the depth pixel. Default 1000. 1000 in a depth pixel values means 1.000 meters
- **min_depth_value_filter**: The minimum accepted value of depth. Lower values will be set to 0
- **max_depth_value_filter**: The maximum accepted value of depth. Higher values will be set to 0
- **downsample**: The downsample that you want to apply to the original image. 1 if not downsample wants to be applied.
- **camera\_fps**: The camera or publishing rate from the depth topic.
- **objective\_fps**: The rate or frequency to publish odometry. If higher that the camera_fps it wont be reach since the camera_fps dictates the limit frequency.
- **ctf_levels**: The levels of pyramid to apply on DIFODO. Default to 5. (NOTE: leave as it is since lowering will affect results and higher values performance and wont give better results)
- **fovh_degrees**: FOV horizontal of the camera
- **fovv_degrees**: FOV vertical of the camera
- **fast_pyramid**: Use a fast pyramid to build the image pyramid. Defautl to false(NOTE: Leave as default otherwise it will fail)
- **use_depth_images_timestamp**: If true it will use the depth images timestamps for the odometry messages being publish, instead of the current time. False otherwise. This is useful when we want to create an evaluation time using the same timestamps that the groundtruth have, so they can be evaluated later by evaluation tools such as EVO. (Use when using with RGBD TUM dataset to create groundtruth files that can be evaluated later)

### ros\_difodo_TUM.launch
Configuration ready to use rosify_difodo with [RGBD SLAM](https://vision.in.tum.de/data/datasets/rgbd-dataset/download) dataset from TUM.

### ros\_difodo_and_realsense_default.launch
This launches rs_rgbd.launch from realsense2_camera and ros_difodo. Useful if you are going to use a realsense camera. It will first start the camera publishing with the default parameters and then difodo.

### ros\_difodo_and_realsense.launch
The same as the previous but it will start the local rs_rgbd.launch within ros_difodo, so local changes and configuration can be applied there.

### rs_rgbd.launch
A copy of the original rs_rgbd.launch from realsense2_camera. Use to change parameters without modifying the original launch file.

### ros_difodo_TUM_evaluation_file
Runs rosify_difodo and odometry_evaluation_file_creator, both configured to work together and with TUM dataset, creating a groundtruth that can be used for comparison later.


### Information visualization: Logs, topics...

#### Logs
You can see the logs or output from the node in several places, the first place is on your terminal. But there are also othero options.

**See the odometry topic in terminal**
```
rostopic echo /difodo/odometry
```

**The ROS logs location is:**
```
~/.ros/log/latest
```

Where latest represents the  lastest node launched.

**rqt\_console**

Open in a terminal:
```
rqt_console
```

If no logs are being displayed just press the red circle to start. When runnning the ros_difodo node you will see the logs there.

#### Rviz

Using:
```rosrun tf view_frames```
You can get a PDF that allows you to see the connections betweens frames for all your rosnodes.

Now with the ros message being publish we can see it with **rviz** for example, which is ros visualizer by default.

To see the odometry message in rviz, we will have to select the desired frame (odom in this case) and then add and odometry topic with the *ADD* button.

First, go to the **Displays->Global Options->Fixed Frame** and select "odom" frame. Also select the frame rate which should match with the odometry messages frequency, which will be the "working\_fps" set in the algorithm by the user.

Secondly, select the **Add** button. There are two ways to add or suscribe to topics. The easiest is use the tab "By topic" and there you will see all the available topics. Search for the odometry topic and add it.
Another way to do it is used the tab "By display type" and select "odometry".

Once the odometry is add, go to  **Odometry->Topic** and select the desired odometry topic, in this case the default topic is /difodo/odometry.

To be able to see the algorithm update properly set the properties **Position Tolerance** and **Angle Tolerance** to 0.01 instead of 0.1, that way the pose will be update on the display with changes of 0.01m instead of 0.1m. The changes will be smoother. Also is good to set **Keep** propery to 1 if you only want to see the current position and not a trajectory.

Also I recommend to suscribe and add the topics depth images and color images, so one can appreciate the movement within the images also.

# 2. odometry_evaluation_file_creator
Check the instrucctions and get the code from [https://github.com/RoboticsLabURJC/2019-tfm-omar-garrido/tree/master/code/odometry_evaluation_file_creator](https://github.com/RoboticsLabURJC/2019-tfm-omar-garrido/tree/master/code/odometry_evaluation_file_creator)

## Requisites
The latest version of **odometry_evaluation_file_creator** has been tested and is currently working on Ubuntu 18.04 LTS but it should work on other platforms. It has been tested with latest [ros-melodic](http://wiki.ros.org/melodic).
It should work with other ros versions like **ros-kinetics and Ubuntu 16.04** but havent been tested yet.

**Note: ros-melodic only supports officially Ubuntu 18.04 while ros-kinetics supports only Ubuntu 16.04**

### CMake
In order to compile and install the vast majority of C++ based softwares CMake is widely used.
Everything has been tested with **version 3.10.2**.
Also lower version will work we recommend to have the latest cmake version for your OS since a lot of packages depend on it to be installed.

### OpenCV
When installing ROS also an OpenCV version is installed in the system. We recommend to use that since it wont give you any headache. But you can always build your OpenCV version and the build ROS from sources.
Ros-melodic comes with OpenCV 3.2.0

### ROS
- If your are using Ubuntu 18.04 install latest ros-melodic following the [official instructions](http://wiki.ros.org/melodic/Installation/Ubuntu).
- **(NOT WORKING)** If your are using Ubuntu 16.04 is better to install ros-kinetics from [official instructions](http://wiki.ros.org/kinetic/Installation/Ubuntu).

**Also you can always try to compile ros from sources in your desired Ubuntu version we do not recommend it.**


## Install as a ros package
**First get the code from github:**
```
git clone https://github.com/RoboticsLabURJC/2019-tfm-omar-garrido.git
```

**Move or copy the package to a catking workspace or your ros workspace**
In my case:
```
cp -r odometry_evaluation_file_creator/ ~/Programs/catkin_ws/src/
```

If you dont know where your catkin workspace is or you havent created one:
- See the [section](https://roboticslaburjc.github.io/2019-tfm-omar-garrido/entries/entry9/#creating-a-launch-file).
- [Create a catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

**To see if the package has been correctly install and is being recognized:**
```
rospack list | grep -i odometry_evaluation_file_creator
```
This command should return the **odometry_evaluation_file_creator** package and its location.

**Build the application, generate the binary node**
```
# Go to your catkin_ws root.
cd  ~/Programs/catkin_ws
# Build
catkin_make
```

The command **catkin_make** will build the packages within your catkin workspace. In order to just generate the desired package:
```
# Once this in done, from now on only this package will be build even with catkin_make
catkin_make --only-pkg-with-deps odometry_evaluation_file_creator
# Be sure to do this to be able to build again all the packages
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
```

To see other options to build packages and manage catkin see this [answer](https://answers.ros.org/question/54178/how-to-build-just-one-package-using-catkin_make/) that also uses:

```
sudo apt-get install python-catkin-tools
```

**All the code above in just one code block for copy paste:**
```
git clone https://github.com/RoboticsLabURJC/2019-tfm-omar-garrido.git
cp -r odometry_evaluation_file_creator/ ~/Programs/catkin_ws/src/
cd  ~/Programs/catkin_ws
catkin_make --only-pkg-with-deps odometry_evaluation_file_creator
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
```

## Usage

```
# For rosify_difodo
roslaunch odometry_evaluation_file_creator TUM_dataset_rosify_difodo.launch.launch

# For SD-SLAM

roslaunch odometry_evaluation_file_creator TUM_dataset_sdslam.launch.launch
```

## Configuration parameters
To configure the package we have the following parameters that are within an YAML file:
- **odometry_topic**: The odometry topic we want to subscribe to. Eg: /sdslam/odometry
- **source_frame**: The source frame of the tf. The frame from which we want to transform, camera center in this case. Eg: openni_camera
- **target_frame**: The target frame that we want to transform to. The absolute reference system, the world. Eg world
- **output_file_dir**: A path to a directory where the output files will be stored.

## Launch files
There are a few launch files which also set some parameters, here is a brief summary of them:

**TUM_dataset_rosify_difodo.launch**
This launch files is intended to be used with rosify_difodo. It loads the configuration YAML from odometry_evaluation_file_creator/launch/config/rosify_difodo_evaluator_config.yaml

**TUM_dataset_sdslam.launch**
This launch files is intended to be used with SD-SLAM. It loads the configuration YAML from odometry_evaluation_file_creator/launch/config/sdslam_evaluator_config.yaml


# 3. SD-SLAM
[SD-SLAM](https://github.com/JdeRobot/SDslam) is one of the best visual SLAM algorithms up to date. Since the goal of this project is to fuse SD-SLAM along with rosdify_difodo Ill provide here the instructions to use SD-SLAM

## Installation
See the instructions on https://github.com/JdeRobot/SDslam


## Usage

### Running SD-SLAM RGBD node with roslaunch
Several launch files has been provided so the node RGBD can be run with those.
In order to run SD-SLAM with the configuration for TUM freidburg1 sequences run

```
roslaunch SD-SLAM sdslam_TUM1.launch
```

In order to run SD-SLAM with the configuration for TUM freidburg1 sequences and odometry_evaluation_file_creation to create a groundtruth file that can be compare with the groundtruths of TUM

```
roslaunch SD-SLAM sdslam_TUM1_evaluation_file.launch
```

# TODO (Things that I still have to worked on):

- [ ] CHECK how to properly change depth FPS for realsense since now my attemps hasnt been successful.
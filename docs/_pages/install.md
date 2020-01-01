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
It should work with other ros versions like **ros-kinetics and Ubuntu 16.04** but havent been tested yet.

**Note: ros-melodic only supports officially Ubuntu 18.04 while ros-kinetics supports only Ubuntu 16.04**

### CMake
In order to compile and install the vast majority of C++ based softwares CMake is widely used.
Everythin has been tested with **version 3.10.2**.
Also lower version will work we recommend to have the latest cmake version for your OS since a lot of packages depend on it to be installed.

### OpenCV
When installing ROS also an OpenCV version is installed in the system. We recommend to use that since it wont give you any headache. But you can always build your OpenCV version and the build ROS from sources.
Ros-melodic comes with OpenCV 3.2.0

### ROS
- If your are using Ubuntu 18.04 install latest ros-melodic following the [official instructions](http://wiki.ros.org/melodic/Installation/Ubuntu).
- If your are using Ubuntu 16.04 is better to install ros-kinetics from [official instructions](http://wiki.ros.org/kinetic/Installation/Ubuntu).

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


## Install
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

The default resolution is 30FPS, to change the resolution that the camera send information you will have to change the launch file or overwrite like this the desired value:

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

# TODO (Things that I still have to worked on):

- [ ] CHECK how to properly change depth FPS for realsense since now my attemps hasnt been successful.
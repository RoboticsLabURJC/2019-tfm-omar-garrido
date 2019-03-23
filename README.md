# 2019-tfm-omar-garrido

This TFM aims to obtain a SLAM algorithm using a RGBD sensor like realsense D435 for a drone, in order to get the location of the drone in the environment and a dense mapping of the environment.

## Index

- [Week 1-4 Reading State of the Art SLAM Papers](#week1)
- [Week 5-7 Realsense D435 and SLAM using RGBD sensors](#week2)
- [Week 8-9 Realsense D435 problems](#week3)
- [Week 10-11 Realsense D435 problems fixed and Range Flow Constrain](#week4)


<a name="week1"></a>
## Week 1-4 Reading State of the Art SLAM Papers

### Step 1
First of all, I started by reading some of the advancements in SLAM algorithms done by the JdeRobot group. I read the doctoral thesis of **Eduardo Perdices** [Técnicas para la localización visual robusta de robots en tiempo real con y sin mapas](https://gsyc.urjc.es/jmplaza/students/phd-eduardo_perdices-2017.pdf).

In this thesis I learn about the basics of SLAM algorithm and the differents approaches that have been made over time to solve the problem of simultaneous location and mapping.

### Step 2
After that, since this is a problem using an RGBD camera, I started reading the TFM of **Alberto Martín** [Autolocalización visual 3D usando mapas
RTAB-Map](https://gsyc.urjc.es/jmplaza/students/tfm-visualslam-alberto_martin-2017.pdf).

In this project the SLAM problem is attacked using a pre-generated 3D map of the environment and using a RGB image to get the location in this map. Also RTAB-map tool is used to generate the 3D map.

### Step 3
Finally I proceed to the installation of the [SD-SLAM](https://github.com/JdeRobot/slam-SD-SLAM) software of the JdeRobot group.


<a name="week2"></a>
## Week 5-7 Realsense D435 and SLAM using RGBD sensors

### Step 1 
Proceed to the installation of ROS-kinetics. Since I have Ubuntu 18 and ROS-Kinetics is only available for Ubuntu 16, I had to reinstall everything on Ubuntu 16.

Learn about ROS platform, filesystem (rosbag), ROS Computation Graph Level (similar to microservices architecture), and start testing the ROS executables examples from that comes with SD-SLAM.

Start working with the realsense D435 RGBD sensor.The objective is to create a simple program that reads information from the realsense and shows it on a window (PyQt + ROS)

After installing the Intel Realsense Drivers for the D435 and the ROS wrapper, I find some problems with the installation of the drivers using the *realsense-viewer* app that allows using the intel realsense drivers to visualize the streams from the Realsense D435. The RGB stream can be perfectly seen but when the Depth stream is show the application freezes and stop working. I have to found out a solution.


### Step 2
Search information related to the SLAM problem using RGBD sensors. There is not too much documents related to SLAM using only depth sensor information, but a lot about RGBD (visual SLAM, using feature extraction on the RGB image and then using the depth information for those features in order to improve the visual SLAM algorithms).

But I found some articles that seems interesting and talk about the SLAM from point of view of depth information:
- **Depth Camera Based Indoor Mobile Robot Localization and Navigation** (RANSAC on the point cloud to find planes on the 3D environment)
- **3D pose estimation and mapping with time-of-flight cameras** (Uses RGBD but also has information and problems of the TOF sensors)
- **Direct Depth SLAM** (the same approach that we are trying to make here)

After reading Direct Depth SLAM, Ive found a lot of techniques such as *range flow constrain equation*, *ICP* and more that Ill try to understand and read more about to be able to implement them and evaluate their performance.


<a name="week3"></a>
## Week 8-9 Realsense D435 problems.
Since the Realsense D435 wasnt working properly using the realsense-realview application I try to figure out what was the problem.

- I did reinstall the realsense 2.0 SDK.
- The ouput of rs-enumerate-devices suggest that the recommended firmware was version 5.10.3. So I downgrade the firmware version, but the problem still persists.
- I found this link in the official web of intel [recommended configurations](https://realsense.intel.com/intel-realsense-downloads/#firmware), and tried the recommended configuration they suggest.
- To discard that there was an error with the SDK, drivers or the Ubuntu 16.04.05 LTS, i tried installing the drivers and SDK for Windows 10.
What I found on Windows 10 was that the problem was there too. So is probably not a problem of the SDK or drivers or OS but with the hardware. 
- Also looking in github I found that the USB type C that connects to the realsense also give some errors to some people and they fix it by inserting the usb to the realsense turning it, since a type C USB can be inserted in both sides. What I found is that on one side the realsense camera is detected as USB 2.0, allowing to record on low resolutions but stable. On the other side of the usb, the camera was detected as USB 3.0, allowing lots of configurations but once the streaming start it will fail and freeze the camera. Same problem in linux but the camera was always detected as USB 3.0 but on one side fewer configurations where allowed. The realsense-viewer application seems more stable on windows anyways as I can record on a resolution of 640x480 at 30 fps without problem with the camera as USB 2.0.

Conclusion: I see a few people on github with similar problems. After all this test I can almost assure that this is probably a hardware related problem and not a software problem. Probably a problem with the USB cable, having not enough bandwidth for example, or a problem with my 3.0 USB, since my  motherboard has an old USB 3.0 but I also have a USB 3.0 expansion PCIe card, so there should not be a problem with that. The last thing that comes to my mind is the camera itself, but since the problems seen are bandwith related I guess I can discard this.

<a name="week4"></a>
## Week 10-11 Realsense D435 problems fixed and Range Flow Constrain.
After I get my hands on a borrowed laptop that had a USB 3.0, i could verify that the problems with the realsense D435 where due to my computer, since on that laptop the realsense-viewer app could work on all resolutions at 30 fps without problems.



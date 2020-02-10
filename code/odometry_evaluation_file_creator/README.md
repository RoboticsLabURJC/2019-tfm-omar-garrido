# odometry_evaluation_file_creator

More detailed instrucions to install and usage on [instructions](https://roboticslaburjc.github.io/2019-tfm-omar-garrido/install/#2-odometry-evaluation-file-creator).

```
# For rosify_difodo
roslaunch odometry_evaluation_file_creator TUM_dataset_rosify_difodo.launch.launch

# For SD-SLAM

roslaunch odometry_evaluation_file_creator TUM_dataset_sdslam.launch.launch
```

## What it does
This ros package allows to apply a publish /tf to the pose of an odometry being publish. This final transform pose along with the odometry timestamp is publish on an output text file that follows the [TUM dataset file format](https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats). So it can be compared with TUM sequences.

## How it works
The node subscribes to a /tf between two frames and to an odometry topic. When the first odometry arrives, the /tf is retrieved and that /tf is the one that is used for the rest on the execution. The /tf in the TUM dataset between the **world** frame and the **openni_camera** express the relation between the world and the camera center. The odometry publish is referenced to the center of the camera, in this case the odom frame. odometry frame and openni_camera frame are the same then.

## Configuration parameters that it uses
To configure the package we have the following parameters that are within an YAML file:
- odometry_topic: The odometry topic we want to subscribe to. Eg: /sdslam/odometry
- source_frame: The source frame of the tf. The frame from which we want to transform, camera center in this case. Eg: openni_camera
- target_frame: The target frame that we want to transform to. The absolute reference system, the world. Eg world
- output_file_dir: A path to a directory where the output files will be stored.


## How to install
To install this node you can follow the instructions from [instructions](https://roboticslaburjc.github.io/2019-tfm-omar-garrido/install/#2-odometry-evaluation-file-creator).
The only differences regarding requisites with rosify_difodo is that this package doesnt require MRPT. (So if you have installed rosify_difodo no more requirements are needed)

To install it can be done as a ROS package or with CMake. I recommend to do it as a ROS package to use the launch files provided.

```
git clone https://github.com/RoboticsLabURJC/2019-tfm-omar-garrido.git
cp -r odometry_evaluation_file_creator/ ~/Programs/catkin_ws/src/
cd  ~/Programs/catkin_ws

## Two options:

# 1. Build all catkin packages
catkin_make

# OR

# 2. Build only the odometry_evaluation_file_creator package
catkin_make --only-pkg-with-deps odometry_evaluation_file_creator
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
```


## How to use
Check [official instructions to see more details of this section](https://roboticslaburjc.github.io/2019-tfm-omar-garrido/install/#2-odometry-evaluation-file-creator).

Use the launch files provided within the package. I provided two launch files:

**TUM_dataset_rosify_difodo.launch**
This launch files is intended to be used with rosify_difodo. It loads the configuration YAML from odometry_evaluation_file_creator/launch/config/rosify_difodo_evaluator_config.yaml

**TUM_dataset_sdslam.launch**
This launch files is intended to be used with SD-SLAM. It loads the configuration YAML from odometry_evaluation_file_creator/launch/config/sdslam_evaluator_config.yaml

Call either of them with:

```
roslaunch odometry_evaluation_file_creator TUM_dataset_rosify_difodo.launch

# OR

roslaunch odometry_evaluation_file_creator TUM_dataset_sdslam.launch
```
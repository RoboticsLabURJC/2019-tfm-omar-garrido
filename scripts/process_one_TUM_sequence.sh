#!/bin/bash

FILEPATH=$1
FREIBURG_DATASET_NUMBER=0

# Check that the FILEPATH is a groundtruth from freiburg and which freidburg is (1,2 or 3)
if [[ $FILEPATH == *"freiburg1"* ]]; then
  FREIBURG_DATASET_NUMBER=1

elif [[ $FILEPATH == *"freiburg2"* ]]; then
  FREIBURG_DATASET_NUMBER=2

elif [[ $FILEPATH == *"freiburg3"* ]]; then
  FREIBURG_DATASET_NUMBER=3
else
  echo "Not a valid freiburg file path."
  exit
fi

# 1. Process rosify_difodo
echo -------------ROSIFY_DIFODO: Process file: $FILEPATH on `date`-------------
roslaunch rosify_difodo ros_difodo_TUM_evaluation_file.launch &
declare -i DIFODO_PID=$!
sleep 2 # Give time to start the ros_node
rosbag play $FILEPATH > /dev/null
ps -p ${DIFODO_PID} | fgrep roslaunch && kill ${DIFODO_PID}


# 2. Process SD-SLAM
echo -------------SD-SLAM RGBD: Process file: $FILEPATH on `date`-------------
roslaunch SD-SLAM sdslam_TUM${FREIBURG_DATASET_NUMBER}_evaluation_file.launch &
declare -i SDSLAM_PID=$!
sleep 2 # Give time to start the ros_node
rosbag play $FILEPATH > /dev/null
ps -p ${SDSLAM_PID} | fgrep roslaunch && kill ${SDSLAM_PID}


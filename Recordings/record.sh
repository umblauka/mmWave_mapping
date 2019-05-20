#!/bin/bash
#TODO:
#   automatically fetch tracker configuration from tracker.cpp/hpp
#       publish the tracker configuration as a topic?
#   different modes for recording: 
#       only pointcloud 
#       pointcloud with tracker markers
#       pointcloud + orientation
#   record the rviz screen using vokoscreen or something other
#

cleanup()
{
  printf "\nCaught Signal ... cleaning up.\n"
  pkill -P $$
  printf  "\nDone cleanup ... quitting.\n"
  exit 1
}

trap cleanup 1 2 3 6
min=$(date +%Y-%m-%d-%H-%M-%S)
mkdir $min
cd $min

cp /home/user/catkin_ws/src/ti_mmwave_rospkg/cfg/6843_3d.cfg .
cp /home/user/catkin_ws/src/mmWave_octomap/src/tracker.cpp .
cp /home/user/catkin_ws/src/mmWave_octomap/include/mmWave_octomap/tracker.hpp .
cp /home/user/catkin_ws/src/ti_mmwave_rospkg/launch/rviz_6843_3d.launch .
cp /home/user/catkin_ws/src/ti_mmwave_rospkg/launch/radar_det_obj_color_by_elev.rviz .

rosbag record /mmWaveDataHdl/RScan /tf /visualization_marker /visualization_marker_array

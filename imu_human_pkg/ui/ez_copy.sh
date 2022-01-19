#!/bin/sh
pyuic5 -x ~/catkin_ws/src/imu_human_pkg/imu_human_pkg/ui/main.ui -o ~/catkin_ws/src/imu_human_pkg/imu_human_pkg/ui/gui.py
cp ~/catkin_ws/src/imu_human_pkg/imu_human_pkg/ui/gui.py ~/catkin_ws/src/imu_human_pkg/imu_human_pkg/src/Classes/gui.py

#!/usr/bin/env python3

import rospy
from Classes.imu_subscriber_class import IMUsubscriber
# from Classes.imu_myo_subscriber_class import IMUsubscriber
from geometry_msgs.msg import Vector3

if __name__ == "__main__": 
	IMU = IMUsubscriber(rate=100, start_node=True)
	IMU.init_subscribers_and_publishers()
	while not rospy.is_shutdown():
		IMU.update()
		IMU.r.sleep()

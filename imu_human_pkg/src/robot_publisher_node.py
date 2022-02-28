#!/usr/bin/env python3

"""
To merge all robot publishers in one node
"""

import rospy
from Classes.robot_publisher_class import RobotPublisher
# from Classes.imu_myo_subscriber_class import IMUsubscriber
from geometry_msgs.msg import Vector3

if __name__ == "__main__": 
	Robot = RobotPublisher(rate=100, start_node=True, name='UR5e')
	Robot.attach_robot()
	Robot.init_subscribers_and_publishers()
	while not rospy.is_shutdown():
		Robot.update()
		Robot.r.sleep()

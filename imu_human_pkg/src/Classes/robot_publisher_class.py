import sys, rospy

from math import pi, cos, sin, sqrt
import numpy as np
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

from rtde_control import RTDEControlInterface as RTDEControl
import rtde_receive

class RobotPublisher:
	def __init__(self, rate=100, start_node=False, name='UR5e'):
		""" Initializes robot communication
		@param name: robot name"""

		self.goal_pose = Pose()

		if start_node == True:
			rospy.init_node("TODO")
			self.r = rospy.Rate(rate)

		if(name == 'UR5e'):
			## Call UR5e node
			self.rtde_c = RTDEControl("172.31.1.144", RTDEControl.FLAG_USE_EXT_UR_CAP)
			self.rtde_r = rtde_receive.RTDEReceiveInterface("172.31.1.144")
			self.goal_pose = self.rtde_r.getActualTCPPose()
			pass #TODO
		elif(name == 'Panda'):
			## Call panda node
			self.goal_pose = self.getCurrentPose()
			pass #TODO

		print("Connection setup. Ready to connect")


	def attach_robot(self):
		pass

	def init_subscribers_and_publishers(self):
		self.pub = rospy.Publisher('/goal_pose', Pose, queue_size=1)
		pass

	def update(self):
		# print self.calibration_flag
		# self.human_joint_imu.header.stamp = rospy.Time.now()
		self.calibration_flag = self.calibration_flag + 1
		self.pub.publish(self.human_joint_imu)
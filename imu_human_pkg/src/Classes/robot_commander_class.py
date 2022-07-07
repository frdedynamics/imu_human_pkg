#! /usr/bin/env python3

"""
Robot Commander
Subscribes human commands, starts UR-RDTE and drives the real UR5e robot in real-time.
"""

import sys, time
import rospy
from math import pi
from math import radians as d2r
import numpy as np

from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Int16, Float64, Bool
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Header

from tf.transformations import quaternion_from_euler as e2q
from tf.transformations import euler_from_quaternion as q2e
from tf.transformations import quaternion_multiply
from tf.transformations import euler_from_matrix
from tf.transformations import quaternion_about_axis

from . import Kinematics_with_Quaternions as kinematic

from rtde_control import RTDEControlInterface as RTDEControl
import rtde_receive


class RobotCommander:
	def __init__(self, rate=100, start_node=False):
		"""Initializes the robot commander"""

		self.rtde_c = RTDEControl("172.31.1.144", RTDEControl.FLAG_USE_EXT_UR_CAP)
		self.rtde_r = rtde_receive.RTDEReceiveInterface("172.31.1.144")

		self.current_TCP_list = Float32MultiArray()
		self.init_list = self.rtde_r.getActualTCPPose()
		self.init_joints = self.rtde_r.getActualQ()

		self.release_joints = [0.9088020324707031, -2.710853715936178, -1.2618284225463867, -2.241020818749899, -1.9657891432391565, -1.429659668599264]
		self.release_approach_joints = [0.7912373542785645, -2.560136457482809, -1.7473573684692383, -1.9465114078917445, -2.089461151753561, -1.5463460127459925]
		self.release_prev_joints = [0.7912373542785645, -2.560136457482809, -1.7473573684692383, -1.9465114078917445, -2.089461151753561, -1.5463460127459925]
		self.home_approach_joints = [1.8097128868103027, -1.9427601299681605, -1.9727983474731445, -2.3609143696227015, -1.35020620027651, -1.5293439070331019]
		self.colift_init_list = 6*[None]
		self.approach_init_list = 6*[None]

		self.gripper_cmd = Bool()

		if start_node == True:
			rospy.init_node("robot_commander")
			self.r = rospy.Rate(rate)
			print("Robot Commander Node Created")

		self.target_pose = Pose()
		self.joints = JointState()
		self.joints.header = Header()
		self.joints.header.frame_id = 'base_link'
		self.joints.name = ['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3']
             

	def init_subscribers_and_publishers(self):
		self.pub_grip_cmd = rospy.Publisher('/cmd_grip_bool', Bool, queue_size=1)
		self.pub_tcp_current_list = rospy.Publisher('/tcp_current_list', Float32MultiArray, queue_size=1)
		self.pub_tcp_current_pose = rospy.Publisher('/tcp_current_pose', Pose, queue_size=1)
		self.pub_robot_joints = rospy.Publisher('/ur5e_joint_state', JointState, queue_size=1)


	####### State methods #######

	def set_approach_init_TCP_pose(self):
		self.approach_init_TCP_list = self.rtde_r.getActualTCPPose()
		
	def set_colift_init_TCP_pose(self):
		self.colift_init = self.rtde_r.getActualTCPPose()

	def move_relative_to_current_pose(self, goal_pose):
		self.rtde_c.servoL(goal_pose,0.5, 0.3, 0.002, 0.1, 300)

	def open_gripper(self):
		self.gripper_cmd.data = False
		self.pub_grip_cmd.publish(self.gripper_cmd)

	def close_gripper(self):
		self.gripper_cmd.data = True
		self.pub_grip_cmd.publish(self.gripper_cmd)
		

	def hrc_idle(self, from_colift=False):
		self.rtde_c.servoStop()
		self.rtde_c.forceModeStop()
		print("IDLE again")
		# if not from_colift:
		# 	# self.robot_pose = self.home_hrc
		# 	self.rtde_c.moveL(self.home_hrc)
		if(self.right_hand_pose.orientation.w > 0.707 and self.right_hand_pose.orientation.x < 0.707): # right rotate downwards
			if not self.hrc_hand_calib_flag:
				print("IDLE calib")
				print("Move to initial arm poses in 4 seconds...")
				rospy.sleep(1)
				print("3 seconds...")
				rospy.sleep(1)
				print("2 seconds...")
				rospy.sleep(1)
				print("1 second...")
				rospy.sleep(1)
				self.call_hand_calib_server()
				self.hrc_hand_calib_flag = True
			if self.hand_grip_strength.data > 75:
				self.status = 'HRC/colift'
			else:
				self.status = 'HRC/approach'
		else:
			self.status = 'HRC/idle'

		return self.status


	def update(self):
		self.pub_grip_cmd.publish(self.gripper_cmd)

		self.current_TCP_list.data = self.rtde_r.getActualTCPPose()
		self.pub_tcp_current_list.publish(self.current_TCP_list)

		self.pub_tcp_current_pose = kinematic.list_to_pose(self.current_TCP_list.data)
		self.pub_tcp_current_pose = rospy.Publisher('/tcp_current_pose', Pose, queue_size=1)

		self.joints.position = self.rtde_r.getActualQ()
		self.joints.header.stamp = rospy.Time.now()		
		self.pub_robot_joints = rospy.Publisher('/ur5e_joint_state', JointState, queue_size=1)
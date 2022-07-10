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
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
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

		self.tcp_force = Float32MultiArray()
		## Skip this for now to reduce the rosbag size. Directly uncomment to use.
		# tcp_layout = MultiArrayDimension()
		# tcp_layout.label = "tcp_force"
		# tcp_layout.size = 1
		# tcp_layout.stride = 6
		# self.tcp_force.layout.dim.append(tcp_layout)
		self.init_list = self.rtde_r.getActualTCPPose()
		self.init_joints = self.rtde_r.getActualQ()
		self.tcp_offset = [0.0, 0.0, 0.14, 0.0, 0.0, 0.0] # self.rtde_c.getTCPOffset() non-responsive. Use it with getForwardKinematics().

		## You can always set it via roscd arm_motion_control_py3/src/Classes/auto_robot_poses.py
		self.home_joints = [1.617659568786621, -1.95817293743276, -2.140641212463379, -2.156917234460348, -1.530267063771383, -1.6029561201678675]
		self.release_joints = [0.7257862091064453, -2.6378027401366175, -1.342545509338379, -2.389071126977438, -2.3516574541675013, -1.6020453611956995]
		self.before_release_joints = [1.0842461585998535, -2.1514698467650355, -1.7598390579223633, -2.4411307773985804, -2.0929248968707483, -1.60185414949526]
		self.after_release_joints = [0.6435627937316895, -2.5409981213011683, -1.622014045715332, -2.1787873707213343, -2.3522325197802942, -1.6022732893573206]
		self.before_home_joints = [0.9844388961791992, -1.9257198772826136, -2.083815574645996, -2.261669775048727, -2.131503407155172, -1.6019018332110804]
		self.colift_init_list = self.rtde_r.getActualTCPPose()
		self.approach_init_TCP_list = self.rtde_r.getActualTCPPose()
		self.current_TCP_list = Float32MultiArray()

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
		self.pub_tcp_force = rospy.Publisher('/tcp_force', Float32MultiArray, queue_size=1)


	####### State methods #######

	def set_approach_init_TCP_pose(self):
		self.approach_init_TCP_list = self.rtde_r.getActualTCPPose()
		
	def set_colift_init_TCP_pose(self):
		self.colift_init = self.rtde_r.getActualTCPPose()

	def move_relative_to_current_pose(self, goal_pose):
		'''
		Think about speed again: https://www.universal-robots.com/articles/ur/application-installation/automove-speed-and-acceleration/

		SERVOJ(Q, A, V, T=0.002, LOOKAHEAD_TIME=0.1, GAIN=300)
		'''
		self.rtde_c.servoL(goal_pose,0.5, 0.3, 0.002, 0.1, 300) # Test a avnd v with Sondre. Set the speed slider to 50% for now.

	def open_gripper(self):
		self.gripper_cmd.data = False
		self.pub_grip_cmd.publish(self.gripper_cmd)
		rospy.sleep(1.5)

	def close_gripper(self):
		self.gripper_cmd.data = True
		self.pub_grip_cmd.publish(self.gripper_cmd)
		rospy.sleep(1.5)
		

	def update(self):
		self.pub_grip_cmd.publish(self.gripper_cmd)

		self.tcp_force.data = self.rtde_r.getActualTCPForce()
		# print(f'0: {_curr_force[0]:.2f} -- 1: {_curr_force[1]:.2f} -- 2: {_curr_force[2]:.2f} -- 3: {_curr_force[3]:.2f} -- 4: {_curr_force[4]:.2f} -- 5: {_curr_force[5]:.2f} -- ')
		self.pub_tcp_force.publish(self.tcp_force)

		self.current_TCP_list.data = self.rtde_r.getActualTCPPose()
		self.pub_tcp_current_list.publish(self.current_TCP_list)

		self.pub_tcp_current_pose = kinematic.list_to_pose(self.current_TCP_list.data)
		self.pub_tcp_current_pose = rospy.Publisher('/tcp_current_pose', Pose, queue_size=1)

		self.joints.position = self.rtde_r.getActualQ()
		self.joints.header.stamp = rospy.Time.now()		
		self.pub_robot_joints = rospy.Publisher('/ur5e_joint_state', JointState, queue_size=1)
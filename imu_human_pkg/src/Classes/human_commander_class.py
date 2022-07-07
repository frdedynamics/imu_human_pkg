#! /usr/bin/env python3

"""
Human Commander
Subscribes two hand poses and sends robot pose command.
"""
from os import stat
import sys, time
import rospy
from math import pi
from math import radians as d2r
import numpy as np

import actionlib
from imu_human_pkg.msg import handCalibrationAction, handCalibrationGoal

from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Int16, Float64, Bool
from std_msgs.msg import Float32MultiArray

from tf.transformations import quaternion_from_euler as e2q
from tf.transformations import euler_from_quaternion as q2e
from tf.transformations import quaternion_multiply
from tf.transformations import euler_from_matrix
from tf.transformations import quaternion_about_axis

from . import Kinematics_with_Quaternions as kinematic


class HumanCommander:
	def __init__(self, rate=100, start_node=False, sr=1.5, sl=1.0, so=2.0):
		"""Initializes the human commander
			@params s: motion hand - steering hand scale
			@params k: target hand pose - robot pose scale"""

		if start_node == True:
			rospy.init_node("human_commander")
			self.r = rospy.Rate(rate)
			print("Human Commander Node Created")

		# print("============ Arm current pose: ", self.rtde_r.getActualTCPPose())
		self.merge_hand_pose = Pose()
		self.left_hand_pose = Pose()
		self.right_hand_pose = Pose()
		self.hand_grip_strength = Int16()

		self.human_to_robot_init_orientation = Quaternion()

		self.elbow_left_height = 0.0
		self.elbow_right_height = 0.0

		self.hand_init_orientation = Quaternion()
		self.human_to_robot_orientation = Quaternion(0.0, 0.0, 0.707, 0.707)
		self.sr = sr # WS scaling right hand
		self.sl = sl # WS scaling left hand 
		self.so = so # Orientation scaling of left wrist to robot wrist

		self.prev_state = ""
		self.state = "IDLE"
		self.role = "HUMAN_LEADING"  # or "ROBOT_LEADING"
		self.hrc_status = String()
		self.status = 'TO/idle'

		self.colift_dir = 'up'
		self.colift_flag = 0
		self.hrc_hand_calib_flag = False
		self.hrc_colift_calib_flag = False
		self.wrist_calib_flag = False

		self.do_flag = 0
               

	def init_subscribers_and_publishers(self):
		self.sub_left_hand_pose = rospy.Subscriber('/left_hand_pose', Pose, self.cb_left_hand_pose)
		self.sub_right_hand_pose = rospy.Subscriber('/right_hand_pose', Pose, self.cb_right_hand_pose)
		self.sub_human_ori = rospy.Subscriber('/human_ori', Quaternion, self.cb_human_ori)
		self.sub_elbow_left = rospy.Subscriber('/elbow_left', Pose, self.cb_elbow_left)
		self.sub_elbow_right= rospy.Subscriber('/elbow_right', Pose, self.cb_elbow_right)
		self.sub_emg_sum= rospy.Subscriber('/emg_sum', Int16, self.cb_emg_sum)
		# self.sub_sensor_lw = rospy.Subscriber('/sensor_l_wrist_rpy', Vector3, self.cb_sensor_lw) # Later with TO

		self.pub_hrc_status = rospy.Publisher('/hrc_status', String, queue_size=1)
		self.pub_colift_dir = rospy.Publisher('/colift_dir', String, queue_size=1)
		self.pub_hands_cmd = rospy.Publisher('/hand_output', String, queue_size=1)

		try:
			self.elbow_height_th = rospy.get_param("/elbow_height_th")
			print("elbow_height_th parameter set:", self.elbow_height_th)
			self.emg_sum_th = rospy.get_param("/emg_sum_th")
			print("emg_sum_th parameter set:",self.emg_sum_th)
		except:
			print("no elbow/emg parameter set")



	####### Callback methods #######

	def cb_emg_sum(self, msg):
		self.hand_grip_strength = msg

	def cb_elbow_left(self, msg):
		self.elbow_left_height = msg.position.z
		
	def cb_elbow_right(self, msg):
		self.elbow_right_height = msg.position.z

	def cb_human_ori(self, msg):
		""" Subscribes chest IMU orientation to map human w.r.t the world frame """
		self.human_to_robot_orientation = kinematic.q_multiply(self.human_to_robot_init_orientation, msg)

	def cb_left_hand_pose(self, msg):
		""" Subscribes left hand pose """
		self.left_hand_pose = msg	

	def cb_right_hand_pose(self, msg):
		""" Subscribes right hand pose """
		self.right_hand_pose = msg

	def call_hand_calib_server(self):
		self.client = actionlib.SimpleActionClient('hand_calibration_as', handCalibrationAction)
		self.client.wait_for_server()
		self.goal = handCalibrationGoal()
		self.goal.calib_request = True
		self.client.send_goal(self.goal, feedback_cb=self.hand_calib_feedback_cb)
		self.client.wait_for_result()
		result = self.client.get_result()

		return result # maybe result is not needed?
	
	def hand_calib_feedback_cb(self, msg):
		print('Hand poses initialized:', msg)

	

	####### HumanClass methods #######

	def get_state(self):
		'''
		Based on gestures, HRC state is determined
		'''

		if not self.prev_state == self.state: ## This makes sure that each state can run a pre-requirements once
			state_transition_flag = True
		else:
			state_transition_flag = False

		if((self.right_hand_pose.orientation.w > 0.707 and self.right_hand_pose.orientation.x < 0.707) and self.hand_grip_strength.data < self.emg_sum_th): # right rotate downwards
			self.state = "APPROACH"
		elif((self.right_hand_pose.orientation.w < 0.707 and self.right_hand_pose.orientation.x > 0.707) and self.hand_grip_strength.data < self.emg_sum_th): # right rotate upwards
			self.state = "IDLE"
		
		elif(self.hand_grip_strength.data > self.emg_sum_th):
			self.status = "COLIFT"

		elif(self.right_hand_pose.position.x < -0.25 and self.right_hand_pose.position.z < -0.15):
			self.status = "RELEASE"

		return self.state, state_transition_flag


	def hands_calib(self):
		'''
		This might be depreciated for button enabled calibration
		'''
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

	
	def hands_reset(self):
		'''
		It is like immediate calibration.
		'''
		reset_flag = False
		reset_flag = self.call_hand_calib_server()
		return reset_flag
	

	def two_hands_move(self, robot_pose):
		'''
		Calculate merge hands pose.
		Move the robot TCP with merged hands command with respect to the previous pose. 
		@params robot_current_pose: list[6]=[position orientation]
		'''
		self.merge_hand_pose.position.x = (- self.left_hand_pose.position.x) - self.sr * self.right_hand_pose.position.x
		self.merge_hand_pose.position.y = (- self.left_hand_pose.position.y) - self.sr * self.right_hand_pose.position.y
		self.merge_hand_pose.position.z = self.left_hand_pose.position.z + self.sr * self.right_hand_pose.position.z
		self.merge_hand_pose.orientation = self.left_hand_pose.orientation

		corrected_merge_hand_pose = kinematic.q_rotate(self.human_to_robot_orientation, self.merge_hand_pose.position)

		robot_goal_pose = 6*[None]
		robot_goal_pose[0] = self.robot_pose[0] + self.sl * corrected_merge_hand_pose[0]
		robot_goal_pose[1] = self.robot_pose[1] - self.sl * corrected_merge_hand_pose[1]
		robot_goal_pose[2] = self.robot_pose[2] + self.sl * corrected_merge_hand_pose[2]
		robot_goal_pose[3:] = self.robot_pose[3:]

		return robot_goal_pose

		

	def hrc_release(self):
		print("Moving to RELEASE pose")
		self.rtde_c.servoStop()
		self.rtde_c.forceModeStop()
		self.rtde_c.moveL(self.release_before)
		self.rtde_c.moveL(self.release)
		cmd_release = Bool()
		cmd_release = False
		self.pub_grip_cmd.publish(cmd_release)
		print("Robot at RELEASE")
		rospy.sleep(4)  # Wait until the gripper is fully open
		self.rtde_c.moveL(self.release_after)
		print("Robot at RELEASE APPROACH")

		self.status = 'HRC/release'


	def update2(self,x):
		try:
			self.teleop_active()
			print(self.robot_init[0]-self.robot_pose[0],
				  self.robot_init[1]-self.robot_pose[1],
				  self.robot_init[2]-self.robot_pose[2])
			self.rtde_c.servoL(self.robot_pose, 0.5, 0.3, 0.01, 0.1, 300)	
			# self.rtde_c.servoL([self.robot_pose[0], self.robot_pose[1], self.robot_pose[2], self.robot_init[3], self.robot_init[4], self.robot_init[5]+x], 0.5, 0.3, 0.01, 0.1, 300)	

			return self.status
		except KeyboardInterrupt:
			self.rtde_c.stopScript()
			raise


	def update(self):
		# State machine here
		status = self.status
		if status == 'TO/idle':
			self.teleop_idle()
		elif status == 'TO/active':
			self.teleop_active()
		elif status == 'HRC/idle':
			self.hrc_idle()
			# self.rtde_c.stopScript()
			# sys.exit()
		elif status == 'HRC/approach':
			self.hrc_approach()
		elif status == 'HRC/colift':
			self.hrc_colift()
		elif status == 'HRC/release':
			self.hrc_release()
			user_input = input("Ready to new cycle?")
			if user_input == 'y':
				self.rtde_c.moveJ(self.home_teleop_approach_joints, speed=0.5)
				self.rtde_c.moveJ(self.home_teleop_joints, speed=0.5)
				self.colift_dir = 'up'
				self.colift_flag = False
				self.hrc_hand_calib_flag = False
				self.hrc_colift_calib_flag = False
				self.wrist_calib_flag = False
				self.status = 'TO/idle'
			else:
				self.status = 'IDLE'
		elif status == 'IDLE':
			print("System is in halt, please restart all the nodes for calibration")
			sys.exit()
		else:
			print("Unknown state:", status)
			sys.exit()
	
		# print("state:", self.state, "    role:", self.role)
		# print("status:", status)
		# self.hrc_status = self.state + ',' + self.role
		# self.hrc_status = self.status

		self.pub_hrc_status.publish(status)
		self.robot_current_TCP.data = self.rtde_r.getActualTCPPose()
		self.pub_tcp_current.publish(self.robot_current_TCP)
		# robot_pose_pose = kinematic.list_to_pose(self.robot_pose)
		# self.pub_tcp_goal.publish(robot_pose_pose)

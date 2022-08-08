#! /usr/bin/env python3

"""
Human Commander
Subscribes two hand poses and sends robot pose command.
"""

import sys
import rospy
import actionlib
from imu_human_pkg.msg import handCalibrationAction, handCalibrationGoal

from geometry_msgs.msg import Pose, Quaternion 
from std_msgs.msg import String, Int16
from std_msgs.msg import Float32MultiArray

from . import Kinematics_with_Quaternions as kinematic


class HumanCommander:
	def __init__(self, rate=100, start_node=False, sr=1.0, sl=1.0, so=2.0):
		"""Initializes the human commander
			@params s: motion hand - steering hand scale
			@params k: target hand pose - robot pose scale"""

		if start_node == True:
			rospy.init_node("human_commander")
			self.r = rospy.Rate(rate)
			print("Human Commander Node Created")


		self.merge_hand_pose = Pose()
		self.corrected_merge_hand_pose = Pose()
		self.corrected_merge_hand_list = Float32MultiArray()
		self.corrected_merge_hand_list.data = 6*[0.0]
		self.left_hand_pose = Pose()
		self.right_hand_pose = Pose()
		self.hand_grip_strength = Int16()

		self.human_to_robot_init_orientation = Quaternion()

		self.elbow_left_height = 0.0
		self.elbow_right_height = 0.0

		self.human_to_robot_orientation = Quaternion(0.0, 0.0, 0.707, 0.707)
		self.sr = sr # WS scaling right hand
		self.sl = sl # WS scaling left hand 
		self.so = so # Orientation scaling of left wrist to robot wrist

		self.prev_state = String()
		self.state = String()
		self.state.data = "IDLE"
		self.role = "HUMAN_LEADING"  # or "ROBOT_LEADING"

		self.prev_colift_dir = String()
		self.colift_dir = String()
		self.colift_dir.data = "u"
		

	def init_subscribers_and_publishers(self):
		self.sub_left_hand_pose = rospy.Subscriber('/motion_hand_pose', Pose, self.cb_left_hand_pose)
		self.sub_right_hand_pose = rospy.Subscriber('/steering_hand_pose', Pose, self.cb_right_hand_pose)
		self.sub_human_ori = rospy.Subscriber('/human_ori', Quaternion, self.cb_human_ori)
		self.sub_elbow_left = rospy.Subscriber('/elbow_left', Pose, self.cb_elbow_left)
		self.sub_elbow_right= rospy.Subscriber('/elbow_right', Pose, self.cb_elbow_right)
		self.sub_emg_sum= rospy.Subscriber('/emg_sum', Int16, self.cb_emg_sum)

		self.pub_hrc_state = rospy.Publisher('/hrc_state', String, queue_size=1)
		self.pub_colift_dir = rospy.Publisher('/colift_dir', String, queue_size=1)
		self.pub_merge_hands = rospy.Publisher('/merged_hands', Pose, queue_size=1)
		self.pub_corr_merge_hands = rospy.Publisher('/corr_merged_hands', Pose, queue_size=1)
		# self.pub_corr_merge_hands_list = rospy.Publisher('/corr_merged_hands_list', Float32MultiArray, queue_size=1)

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
		self.elbow_left_height = msg.position.y
		
	def cb_elbow_right(self, msg):
		self.elbow_right_height = -msg.position.y

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

		if((self.hand_grip_strength.data > self.emg_sum_th) and self.state.data == "APPROACH"):
			self.state.data = "COLIFT"
			rospy.set_param("/colift_set", True)

		elif(((self.right_hand_pose.orientation.w < 0.707 and self.right_hand_pose.orientation.x > 0.707) and self.hand_grip_strength.data < self.emg_sum_th)and self.state.data != "COLIFT"): # right rotate downwards
			self.state.data = "APPROACH"
		elif((self.right_hand_pose.orientation.w > 0.707 and self.right_hand_pose.orientation.x < 0.707) and self.hand_grip_strength.data < self.emg_sum_th): # right rotate upwards
			self.state.data = "IDLE"

		elif((self.right_hand_pose.position.x < -0.25 and self.right_hand_pose.position.z < -0.15)and self.state.data == "COLIFT"):
			self.state.data = "RELEASE"


		if not self.prev_state == self.state: ## This makes sure that each state can run a pre-requirements once
			state_transition_flag = True
			print("right: ", self.right_hand_pose.orientation.w)
			print("strength: ", self.hand_grip_strength.data, "-", self.emg_sum_th)
			print(self.prev_state, "--", self.state)
		else:
			state_transition_flag = False
		
		self.prev_state.data = self.state.data

		return self.state, state_transition_flag


	def hands_calib(self):
		'''
		This might be depreciated for button enabled calibration. The button uses non-action-client version with wrist_to_robot_2arms.py in arm package.
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

		corrected_merge_hand_list = 6*[0.0]
		corrected_merge_hand_list = kinematic.q_rotate(self.human_to_robot_orientation, self.merge_hand_pose.position)

		robot_goal_pose = 6*[None]
		robot_goal_pose[0] = robot_pose[0] + self.sl * corrected_merge_hand_list[0]
		robot_goal_pose[1] = robot_pose[1] - self.sl * corrected_merge_hand_list[1]
		robot_goal_pose[2] = robot_pose[2] + self.sl * corrected_merge_hand_list[2]
		robot_goal_pose[3:] = robot_pose[3:]

		self.corrected_merge_hand_list.data = corrected_merge_hand_list
		self.corrected_merge_hand_pose = kinematic.list_to_pose(corrected_merge_hand_list)

		return robot_goal_pose


	def get_dir_from_elbows(self, force):
		'''
		Sets direction for compliance force based on elbow heights
		'''

		print("elbow_right_height: ", self.elbow_right_height)
		print("elbow_left_height: ", self.elbow_left_height)
		print("current colift_dir: ", self.colift_dir.data)

		if (self.colift_dir.data == "s" and force < 0):
			if((self.elbow_right_height > self.elbow_height_th) and (self.elbow_left_height < self.elbow_height_th)):
				self.colift_dir.data = "r"
			elif((self.elbow_left_height > self.elbow_height_th) and (self.elbow_right_height < self.elbow_height_th)):
				self.colift_dir.data = "l"
			elif((self.elbow_left_height > self.elbow_height_th) and (self.elbow_right_height > self.elbow_height_th)):
				self.colift_dir.data = "u"
			else:
				print("Something wrong colift 1: ", self.colift_dir.data, '--', force)
				print("left:", self.elbow_left_height)
				print("right:", self.elbow_right_height)
		elif (self.colift_dir.data == "s" and force > 0):
			self.colift_dir.data = "d"
		else:
			self.colift_dir.data = "s"


		if not self.prev_colift_dir == self.colift_dir:
			dir_change_flag = True
		else:
			dir_change_flag = False
			print("direction changed to: ", self.colift_dir.data)
		
		self.prev_colift_dir.data = self.colift_dir.data
		
		return self.colift_dir.data, dir_change_flag
		

	def update(self):
		self.pub_hrc_state.publish(self.state)
		self.pub_colift_dir.publish(self.colift_dir)
		self.pub_merge_hands.publish(self.merge_hand_pose)
		self.pub_corr_merge_hands.publish(self.corrected_merge_hand_pose)
		# print(self.corrected_merge_hand_list)
		# print(type(self.corrected_merge_hand_list))
		# self.pub_corr_merge_hands_list.publish(self.corrected_merge_hand_list)

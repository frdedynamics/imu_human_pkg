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
		self.target_pose = Pose()
		self.left_hand_pose = Pose()
		self.hand_grip_strength = Int16()
		self.right_hand_pose = Pose()
		self.robot_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

		self.elbow_left_height = 0.0
		self.elbow_right_height = 0.0

		self.hand_init_orientation = Quaternion()
		self.human_to_robot_init_orientation = Quaternion(0.0, 0.0, 0.707, 0.707)
		self.sr = sr # WS scaling right hand
		self.sl = sl # WS scaling left hand 
		self.so = so # Orientation scaling of left wrist to robot wrist

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
		self.sub_hand_grip_strength = rospy.Subscriber('/robotiq_grip_gap', Int16, self.cb_hand_grip_strength)
		self.sub_left_hand_pose = rospy.Subscriber('/left_hand_pose', Pose, self.cb_left_hand_pose)
		self.sub_right_hand_pose = rospy.Subscriber('/right_hand_pose', Pose, self.cb_right_hand_pose)
		self.sub_human_ori = rospy.Subscriber('/human_ori', Quaternion, self.cb_human_ori)
		self.sub_sensor_lw = rospy.Subscriber('/sensor_l_wrist_rpy', Vector3, self.cb_sensor_lw)
		self.sub_elbow_left = rospy.Subscriber('/elbow_left', Pose, self.cb_elbow_left)
		self.sub_elbow_right= rospy.Subscriber('/elbow_right', Pose, self.cb_elbow_right)

		self.pub_hrc_status = rospy.Publisher('/hrc_status', String, queue_size=1)
		self.pub_colift_dir = rospy.Publisher('/colift_dir', String, queue_size=1)
		self.pub_hands_cmd = rospy.Publisher('/hand_output', String, queue_size=1)




	####### Callback methods #######

	def cb_elbow_left(self, msg):
		self.elbow_left_height = msg.position.z

		
	def cb_elbow_right(self, msg):
		self.elbow_right_height = msg.position.z


	def cb_sensor_lw(self, msg):
		""" Subscribes the pure IMU RPY topic to control robot TCP orientation"""
		if not self.wrist_calib_flag:
			self.tcp_ori_init.x = msg.x
			self.tcp_ori_init.y = msg.y
			self.tcp_ori_init.z = msg.z
			self.wrist_calib_flag = True
		self.tcp_ori.x = d2r(msg.x-self.tcp_ori_init.x)
		self.tcp_ori.z = d2r(msg.y-self.tcp_ori_init.y)
		self.tcp_ori.y = d2r(msg.z-self.tcp_ori_init.z)  ## This didn't give good results


	def cb_human_ori(self, msg):
		""" Subscribes chest IMU orientation to map human w.r.t the world frame """
		self.human_to_robot_init_orientation = kinematic.q_multiply(Quaternion(0.0, 0.0, 0.707, 0.707), msg)


	def cb_hand_grip_strength(self, msg):
		""" Subscribes hand grip strength
		Open: 0
		Close: 255 """
		self.hand_grip_strength = msg

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


	####### State methods #######

	def teleop_idle(self, from_teleop_active = False):
		# self.robot_pose = self.home_teleop  # For some reason this updates home_teleop 
		if (self.right_hand_pose.orientation.w > 0.707 and self.right_hand_pose.orientation.x < 0.707):
			self.status = 'TO/active'
		else:
			self.status = 'TO/idle'
		return self.status


	def teleop_active(self):
		self.target_pose.position.x = - 1 * self.right_hand_pose.position.x
		self.target_pose.position.y = - 1 * self.right_hand_pose.position.y
		self.target_pose.position.z = 1 * self.right_hand_pose.position.z

		corrected_target_pose = kinematic.q_rotate(self.human_to_robot_init_orientation, self.target_pose.position)
		self.robot_pose[0] = self.home_teleop[0] + self.sl * corrected_target_pose[0]
		self.robot_pose[1] = self.home_teleop[1] - self.sl * corrected_target_pose[1]
		self.robot_pose[2] = self.home_teleop[2] + self.sl * corrected_target_pose[2]
		self.robot_pose[3:] = self.home_teleop[3:]

		joints = self.rtde_c.getInverseKinematics(self.robot_pose)
		joints[3] -= self.tcp_ori.z
		joints[4] -= self.tcp_ori.x
		joints[5] -= self.tcp_ori.y
		self.rtde_c.servoJ(joints,0.5, 0.3, 0.002, 0.1, 300)


		print(self.tcp_ori.x, self.tcp_ori.y, self.tcp_ori.z)
		if (self.right_hand_pose.orientation.w < 0.707 and self.right_hand_pose.orientation.x > 0.707): # right rotate upwards
			if (self.tcp_ori.z > 1.0):
				self.rtde_c.servoStop()
				self.rtde_c.moveL(self.hrc_appr, speed=0.5)
				self.rtde_c.moveJ(self.hrc_appr2_joints, speed=0.5)
				self.rtde_c.moveJ(self.home_hrc_joints, speed=0.5)
				self.status = 'HRC/idle'
			else:
				self.status = 'TO/idle'
		else:
			self.status = 'TO/active'

		return self.status


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


	def hrc_approach(self):
		''' old cartesian_2_arms here 
		robot.init is updated to home_hrc'''
		self.target_pose.position.x = (- self.left_hand_pose.position.x) - self.sr * self.right_hand_pose.position.x
		self.target_pose.position.y = (- self.left_hand_pose.position.y) - self.sr * self.right_hand_pose.position.y
		self.target_pose.position.z = self.left_hand_pose.position.z + self.sr * self.right_hand_pose.position.z
		self.target_pose.orientation = self.left_hand_pose.orientation

		corrected_target_pose = kinematic.q_rotate(self.human_to_robot_init_orientation, self.target_pose.position)
		self.robot_pose[0] = self.home_hrc[0] + self.sl * corrected_target_pose[0]
		self.robot_pose[1] = self.home_hrc[1] - self.sl * corrected_target_pose[1]
		self.robot_pose[2] = self.home_hrc[2] + self.sl * corrected_target_pose[2]
		self.robot_pose[3:] = self.home_hrc[3:]

		self.rtde_c.servoL(self.robot_pose,0.5, 0.3, 0.002, 0.1, 300)

		if(self.right_hand_pose.orientation.w < 0.707 and self.right_hand_pose.orientation.x > 0.707): # right rotate upwards
			self.rtde_c.servoStop()
			self.status = 'HRC/idle'
		
		elif(self.hand_grip_strength.data > 75):
			if not self.hrc_colift_calib_flag:
				# self.call_hand_calib_server()
				self.rtde_c.servoStop()
				self.robot_colift_init = self.rtde_r.getActualTCPPose()
				self.hrc_colift_calib_flag = True
			self.status = 'HRC/colift'

		else:
			self.status = 'HRC/approach'

	def hrc_colift(self):
		f = 15
		# TODO: any problem coming from IDLE but not from APPROACH?
		''' Make force thingy here '''
		# vector = self.rtde_r.getActualTCPPose() # A pose vector that defines the force frame relative to the base frame.
		vector = [0, 0, 0, 0, 0, 0]
		selection_vector = [1, 1, 1, 0, 0, 0] # A 6d vector of 0s and 1s. 1 means that the robot will be compliant in the corresponding axis of the task frame
		wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # The forces/torques the robot will apply to its environment. The robot adjusts its position along/about compliant axis in order to achieve the specified force/torque. Values have no effect for non-compliant axes
		type = 2 # An integer [1;3] specifying how the robot interprets the force frame. 1: The force frame is transformed in a way such that its y-axis is aligned with a vector pointing from the robot tcp towards the origin of the force frame. 2: The force frame is not transformed. 3: The force frame is transformed in a way such that its x-axis is the projection of the robot tcp velocity vector onto the x-y plane of the force frame.
		limits = [f, f, f, 0.17, 0.17, 0.17]# (Float) 6d vector. For compliant axes, these values are the maximum allowed tcp speed along/about the axis. For non-compliant axes, these values are the maximum allowed deviation along/about an axis between the actual tcp position and the one set by the program.

		_curr_force = self.rtde_r.getActualTCPForce()
		# print(_curr_force[0])

		if self.colift_dir == 'right':
			# print(self.status)
			# vector = self.rtde_r.getActualTCPPose()
			selection_vector = [1, 0, 0, 0, 0, 0] 
			wrench = [f, 0.0, 0.0, 0.0, 0.0, 0.0]
			limits = [0.5, 0.3, 0.3, 0.17, 0.17, 0.17]

		elif self.colift_dir == 'left':
			# print(self.status)
			# vector = self.rtde_r.getActualTCPPose()
			selection_vector = [1, 0, 0, 0, 0, 0]
			wrench = [-f, 0.0, 0.0, 0.0, 0.0, 0.0]
			limits = [0.5, 0.3, 0.3, 0.17, 0.17, 0.17]

		elif self.colift_dir == 'up':
			# print(self.status)
			# vector = self.rtde_r.getActualTCPPose()
			selection_vector = [0, 0, 1, 0, 0, 0]
			wrench = [0.0, 0.0, f, 0.0, 0.0, 0.0]
			limits = [0.3, 0.3, 0.5, 0.17, 0.17, 0.17]

		# print("FORCE:", _curr_force[0])		
		if abs(_curr_force[0]) > 1.6:
			# print(_curr_force[0])
		# if self.tcp_ori.z > 1.0:
			# print("Side movement")
			print(self.elbow_right_height, self.elbow_left_height)
			height_th = 0.08
			print(self.colift_dir)
			# print(_curr_force[0])
			# colift_dir = ''
			# colift_dir_past = ''
			if((self.elbow_right_height > height_th) and (self.elbow_left_height < height_th)):
				self.colift_dir = 'left'
				# self.colift_flag = 0
			elif((self.elbow_left_height > height_th) and (self.elbow_right_height < height_th)):
				self.colift_dir = 'right'
				# self.colift_flag = 0
			elif((self.elbow_left_height > height_th) and (self.elbow_right_height > height_th)):
				self.colift_dir = 'up'
				# self.colift_flag = 0
			else:
				if self.colift_flag < 4:
					self.colift_flag += 1
					print(self.colift_flag)
				else:
					self.colift_dir = 'null'
				
		# print(_curr_force[0])
		# check if still in colift
		if(self.right_hand_pose.position.x < -0.25 and self.right_hand_pose.position.z < -0.15):
			# self.rtde_c.forceModeStop()
			self.status = 'HRC/release'
			print('HRC/release')
		elif(self.right_hand_pose.orientation.w < 0.707 and self.right_hand_pose.orientation.x > 0.707):
			# self.rtde_c.forceModeStop()
			self.status = 'HRC/idle'
			print('HRC/idle')
			# self.hrc_idle(from_colift=True)
		elif(self.hand_grip_strength.data < 75):
			self.rtde_c.forceModeStop()
			self.status = 'HRC/approach'
		else:
			self.status = 'HRC/colift'
		# self.robot_pose = self.rtde_c.getTargetWaypoint()
		# bool = self.rtde_c.isSteady()
		# std::vector<double> = self.rtde_r.getActualToolAccelerometer()
		# print(self.colift_dir)
		self.rtde_c.forceMode(vector, selection_vector, wrench, type, limits)

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

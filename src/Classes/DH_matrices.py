import sys

from math import pi, cos, sin, sqrt
import numpy as np
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion


from tf.transformations import quaternion_from_matrix as m2q
from tf.transformations import quaternion_matrix as q2m


class DHmatrices:
	def __init__(self):
		""" Initializes 3 rotm and 3 htm and 3 link_vec
		@param number: number of IMUs. 2 for elbow only, 3 for arm only, 4 for include chest"""

		# self.rotm1 = np.zeros((3, 3), dtype = float)
		# self.rotm2 = np.zeros((3, 3), dtype = float)
		# self.rotm3 = np.zeros((3, 3), dtype = float)
		# link_vec1 = np.zeros(3, dtype = float)
		# link_vec2 = np.zeros(3, dtype = float)
		# link_vec3 = np.zeros(3, dtype = float)
		# self.htm1 = np.zeros((4, 4), dtype = float)
		# self.htm2 = np.zeros((4, 4), dtype = float)
		# self.htm3 = np.zeros((4, 4), dtype = float)


		self.rotm = np.zeros((3, 3), dtype = float)
		self.link_vec = np.zeros((3, 1), dtype = float)
		self.htm = np.zeros((4, 4), dtype = float)


		print("Matrices created")


	def angle_to_rotm(self, theta, alpha):
		'''
		https://blog.robotiq.com/how-to-calculate-a-robots-forward-kinematics-in-5-easy-steps
		'''
		self.rotm[0][0] = cos(theta)
		self.rotm[0][1] = -sin(theta) * cos(alpha)
		self.rotm[0][2] = sin(theta) * sin(alpha)
		self.rotm[1][0] = sin(theta)
		self.rotm[1][1] = cos(theta) * cos(alpha)
		self.rotm[1][2] = -cos(theta) * sin(alpha)
		self.rotm[2][1] = sin(alpha)
		self.rotm[2][2] = cos(alpha)
		return self.rotm
	
	
	def link_calculate(self, theta, a, d):
		'''
		https://blog.robotiq.com/how-to-calculate-a-robots-forward-kinematics-in-5-easy-steps
		'''
		self.link_vec[0] = a * cos(theta)
		self.link_vec[1] = a * sin(theta)
		self.link_vec[2] = d
		return self.link_vec
	
		
	def rotm_to_htm(self, rotm, link_vec):
		null_mat = np.concatenate((rotm, link_vec), axis=1)
		null_vec = np.zeros((1,4), dtype = float)
		null_vec[0][3] = 1.0
		self.htm = np.concatenate((null_mat, null_vec))
		return self.htm
		
				
	@staticmethod	
	def matmul(mat1, mat2, mat3):
		return np.matmul(mat1, np.matmul(mat2, mat3))

	@staticmethod
	def htm_to_rotm(htm):
		return htm[np.ix_([0,3],[0,3])]
		
	@staticmethod
	def htm_to_quat(htm):
		quat = m2q(htm) # np.array([x, y, z, w])
		return quat

	@staticmethod
	def quat_to_htm(quat):
		htm = q2m(quat) # np.array([x, y, z, w])
		return htm
		
	@staticmethod
	def htm_to_vec(htm):
		# vec = htm[:3, 3:4]
		vec = htm[:3, 3]
		return vec
		
	@staticmethod
	def pose_to_htm(pose):
		pose_quat = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
		pose_rotm = q2m(pose_quat) # actually 4x4 matrix created (a htm)
		pose_link = np.array([pose.position.x, pose.position.y, pose.position.z])
		pose_rotm[:3, 3] = pose_link
		return pose_rotm
		
	@staticmethod
	def htm_to_pose(htm):
		htm_quat = m2q(htm)
		pose_quat = Quaternion(htm_quat[0], htm_quat[1], htm_quat[2], htm_quat[3])
		pose_point = Point(htm[0,3], htm[1,3], htm[2,3])
		pose = Pose(pose_point, pose_quat)
		return pose
		
	@staticmethod
	def to_numpy(msg):
		if type(msg) == Point:
			result = np.array([msg.x, msg.y, msg.z])
		elif type(msg) == Quaternion:
			result = np.array([msg.x, msg.y, msg.z, msg.w])
		elif type(msg) == Pose:
			# basically pose_to_htm but I don't want to change it from static method
			pose_quat = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
			pose_rotm = q2m(pose_quat) # actually 4x4 matrix created (a htm)
			pose_link = np.array([msg.position.x, msg.position.y, msg.position.z])
			pose_rotm[:3, 3] = pose_link
			result = pose_rotm
		else:
			print("unknown type")
			result = None
		return result
		
	@staticmethod
	def ee_goal_calculate(hand_pose, wrist_pose, r = 0.248050394992, param='c'):  # r calculated from ik
		'''
		Given hand_pose {Pose()}, parametrizes {z} such that it will be on UR5's workspace sphere.
		@returns ee_goal {np.array(4,4)} - then this will be used as Tee for the robot
		
		Sphere: (x-a)^2 + (y-b)^2 + (z-c)^2 = r2
						(a, b, c): Origin of the sphere -> Wrist origin
						(x, y, z): EE coordinates -> /tool0 position
						r: radius of the workspace
		'''
		ee_goal_htm = DHmatrices.to_numpy(hand_pose)
		wrist_htm = DHmatrices.to_numpy(wrist_pose)
		x, y, z = DHmatrices.htm_to_vec(ee_goal_htm)
		a, b, c = DHmatrices.htm_to_vec(wrist_htm)
		if param == 'x':
			param_x = sqrt(r**2 - (z-c)**2 - (y-b)**2) + a
			return param_x
		elif param == 'y':
			param_y = sqrt(r**2 - (x-a)**2 - (z-c)**2) + b
			return param_y
		elif param == 'z':
			param_z = sqrt(r**2 - (x-a)**2 - (y-b)**2) + c
			return param_z
		else:
			print("Unknown parameter selection")
			return 0
		
		

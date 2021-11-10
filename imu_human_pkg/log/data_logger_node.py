#!/usr/bin/env python3

"""This is the script that works as a ROS node.
		TODO: make it as gui as you did before.
"""

import rospy, time
import Data.data_logger_class as data_logger
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16, String, Float32MultiArray 

# from get_model_gazebo_pose import GazeboModel

lhand_pose = Pose()
rhand_pose = Pose()
lelbow_height = Pose()
relbow_height = Pose()
hand_pose = Pose()
grip_strength = Int16()
tcp_current = Float32MultiArray()
status = String()


def callback_lhand_pose(msg):
	global lhand_pose
	lhand_pose = msg	

def callback_rhand_pose(msg):
	global rhand_pose
	rhand_pose = msg	

def callback_hand_pose(msg):
	global hand_pose
	hand_pose = msg	

def callback_lelbow_pose(msg):
	global lelbow_height
	lelbow_height = msg.position.z

def callback_relbow_pose(msg):
	global relbow_height
	relbow_height = msg.position.z

def callback_tcp(msg):
	global tcp_current
	tcp_current = msg.data	

def callback_grip_strength(msg):
	global grip_strength
	grip_strength = msg.data	

def callback_hrc_status(msg):
	global status
	status = msg.data	

	

if __name__ == "__main__":
	# global lhand_pose, rhand_pose, hand_pose, tgoal_pose, tactual_pose, tactual_corrected_pose
	try:
		rospy.init_node('data_logger_node')
		start_time = time.time()
		current_time = rospy.get_time()
		### getparam behaves like another thread for some reason...
		# if not rospy.has_param('ref_frame'):
		# 	print("Reference frame parameter has not set. Exiting...")
		# 	sys.exit()
		# else:
		# 	param = rospy.get_param('ref_frame')
		# 	print('here')
		username = input("Please enter username: ")
		ref_frame = input("Please enter reference frame: ")
		data_logger.enable_logging(username,ref_frame)

		sub_lhand_pose = rospy.Subscriber('/wrist_left', Pose, callback_lhand_pose)
		sub_rhand_pose = rospy.Subscriber('/wrist_right', Pose, callback_rhand_pose)
		sub_hand_pose = rospy.Subscriber('/hand_pose', Pose, callback_hand_pose)
		sub_elbow_left = rospy.Subscriber('/elbow_left', Pose, callback_lelbow_pose)
		sub_elbow_right= rospy.Subscriber('/elbow_right', Pose, callback_relbow_pose)
		sub_tcp = rospy.Subscriber('/tcp_current', Float32MultiArray, callback_tcp)
		sub_grip_strength = rospy.Subscriber('/robotiq_grip_gap', Int16, callback_grip_strength)
		sub_hrc_status = rospy.Subscriber('/hrc_status', String, callback_hrc_status) 

		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			elapsed_time = time.time() - start_time
			data_logger.log_metrics(elapsed_time, lhand_pose, rhand_pose, hand_pose, lelbow_height, relbow_height, grip_strength, tcp_current, status)
			rate.sleep()
	except KeyboardInterrupt:
		data_logger.disable_logging()
		rospy.signal_shutdown("KeyboardInterrupt")
		raise
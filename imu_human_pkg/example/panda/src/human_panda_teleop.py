#! /usr/bin/env python3

"""
It looks like I will use move_groups for EE mapping and action server for joint space mapping
TODO: This whole thing can be written in classes. My_Move_Groups(). Don't overcode yet :D
Refer to: http://docs.ros.org/en/indigo/api/pr2_moveit_tutorials/html/planning/scripts/doc/move_group_python_interface_tutorial.html
"""

import sys
import time
import copy
from math import pi

import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

# TODO: relative path!
sys.path.append('/home/gizem/catkin_ws/src/imu_human_pkg/imu_human_pkg/src/Classes')
import Kinematics_with_Quaternions as kinematic


GOAL_POSE = Pose()
t = TransformStamped()
target_pose = Pose()
human_to_robot_init_orientation = Quaternion(0,0,0,1)
hand_init_orientation = Quaternion()
motion_hand_pose = Pose()
steering_hand_pose = Pose()
merged_hands_pose = Pose()
s = 1.0
k = 1.0

init_flag = False


def movegroup_init():
	"""
	Initializes the manipulator and end-effector groups
	@returns Initialized groups
	"""
	moveit_commander.roscpp_initialize(sys.argv)
	robot = moveit_commander.RobotCommander()

	rospy.init_node('moveit_move')
	print("hands_calibrate node started")
	arm_group = moveit_commander.MoveGroupCommander("arm")
	arm_group.set_named_target("home")
	plan_arm = arm_group.go()  
	# arm_group.execute(plan_arm, wait=True)  -- For some reason throws an error and no need

	return arm_group

    
def cartesian_control_2_arms():	
	global target_pose, merged_hands_pose, human_to_robot_init_orientation
	target_pose.position.x = (- motion_hand_pose.position.x) - s * steering_hand_pose.position.x
	target_pose.position.y = (- motion_hand_pose.position.y) - s * steering_hand_pose.position.y
	target_pose.position.z = motion_hand_pose.position.z + s * steering_hand_pose.position.z
	target_pose.orientation = motion_hand_pose.orientation

	# print "robot_pose:", robot_pose.position
	corrected_target_pose = kinematic.q_rotate(human_to_robot_init_orientation, target_pose.position)
	robot_pose = Pose()
	merged_hands_pose.position.x = k * corrected_target_pose[0]
	merged_hands_pose.position.y = k * corrected_target_pose[1]
	merged_hands_pose.position.z = k * corrected_target_pose[2]
	robot_pose.orientation = robot_init.orientation

	# pub_merged_hands.publish(merged_hands_pose)

	motion_hand_colift_init = motion_hand_pose
	print(target_pose)
	return merged_hands_pose
		

def cartesian_control_with_IMU(arm_group, robot_init, hand_pose, *argv):
	waypoints = []
	scale = 1.0
	wpose = Pose()
	wpose.position.x = robot_init.position.x + scale * hand_pose.position.y
	wpose.position.y = robot_init.position.y - scale * hand_pose.position.z
	wpose.position.z = robot_init.position.z + scale * hand_pose.position.x
	wpose.orientation = robot_init.orientation
	# wpose.orientation = kinematic.q_multiply(robot_init.orientation, hand_pose.orientation)
	
	waypoints.append(copy.deepcopy(wpose))
	
	(plan, fraction) = arm_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.001,        # eef_step
                                   0.0)         # jump_threshold

	arm_group.execute(plan, wait=True)
	arm_group.stop()
	arm_group.clear_pose_targets()


## CALLBACKS ##

def cb_human_ori(msg):
	""" Subscribes chest IMU orientation to map human w.r.t the world frame """
	global human_to_robot_init_orientation
	human_to_robot_init_orientation = kinematic.q_multiply(Quaternion(0.0, 0.0, 0.707, 0.707), msg)

def cb_hand_grip_strength(msg):
	""" Subscribes hand grip strength
	Open: 0
	Close: 255 """
	global hand_grip_strength
	hand_grip_strength = msg


def cb_motion_hand_pose(msg):
	""" Subscribes left hand pose """
	global init_flag, hand_init_orientation, motion_hand_pose
	motion_hand_pose = msg
	if not init_flag:
		hand_init_orientation = kinematic.q_invert(steering_hand_pose.orientation)
		print("Hand init set:", hand_init_orientation)
		init_flag = True


def cb_steering_pose(msg):
	""" Subscribes right hand pose """
	global steering_hand_pose
	steering_hand_pose = msg

	    

if __name__ == '__main__': 
	movegroup_init()
    
	try:
		arm_group = movegroup_init()		
		robot_init = arm_group.get_current_pose().pose
		print("============ Arm current pose: ", robot_init)
		sub_motion_hand_pose = rospy.Subscriber('/motion_hand_pose', Pose, cb_motion_hand_pose)
		sub_steering_pose = rospy.Subscriber('/steering_hand_pose', Pose, cb_steering_pose)
		sub_human_ori = rospy.Subscriber('/human_ori', Quaternion, cb_human_ori)
		# print("click Enter to continue")
		# dummy_input = input()
		prev = time.time()
		rate = rospy.Rate(125)
		robot_init = arm_group.get_current_pose().pose
		while not rospy.is_shutdown():
			merged_hands = cartesian_control_2_arms()
			cartesian_control_with_IMU(arm_group, robot_init, merged_hands)
		rate.sleep()

	except KeyboardInterrupt:
		moveit_commander.roscpp_shutdown()
		rospy.signal_shutdown("KeyboardInterrupt")
		raise

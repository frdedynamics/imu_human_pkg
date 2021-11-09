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

sys.path.append('/home/gizem/catkin_ws/src/imu_human_pkg/src/')
from Classes.imu_subscriber_class import IMUsubscriber
import Classes.Kinematics_with_Quaternions as kinematic


# EE_POSE = Odometry()
# WRIST_POSE = Odometry()
# GOAL_POSE = Pose()
# t = TransformStamped()

# IMU = IMUsubscriber()
# #TODO: initiate human model


# def movegroup_init():
# 	"""
# 	Initializes the manipulator and end-effector groups
# 	@returns Initialized groups
# 	"""
# 	moveit_commander.roscpp_initialize(sys.argv)
# 	robot = moveit_commander.RobotCommander()

# 	arm_group = moveit_commander.MoveGroupCommander("manipulator")
# 	arm_group.set_named_target("home")
# 	plan_arm = arm_group.go()  
# 	return arm_group

    
# def task_space_control(arm_group, *argv):
# 	global EE_POSE
# 	# rosrun tf tf_echo /world /tool0
# 	pose_goal = Pose()
# 	pose_goal.orientation.x = 0.707
# 	pose_goal.orientation.y = -0.000
# 	pose_goal.orientation.z = -0.000
# 	pose_goal.orientation.w = 0.707
# 	pose_goal.position.x = argv[0]
# 	pose_goal.position.y = argv[1]
# 	pose_goal.position.z = argv[2]
# 	arm_group.set_pose_target(pose_goal)

# 	## Now, we call the planner to compute the plan and execute it.
# 	plan = arm_group.go(wait=True)
# 	# Calling `stop()` ensures that there is no residual movement
# 	arm_group.stop()
# 	# It is always good to clear your targets after planning with poses.
# 	# Note: there is no equivalent function for clear_joint_value_targets()
# 	arm_group.clear_pose_targets()

# 	## END_SUB_TUTORIAL

# 	# For testing:
# 	# Note that since this section of code will not be included in the tutorials
# 	# we use the class variable rather than the copied state variable
# 	current_pose = arm_group.get_current_pose().pose
# 	print "EE_POSE", EE_POSE
# 	# return all_close(pose_goal, current_pose, 0.01)
	
# def cartesian_control(arm_group, *argv):
# 	waypoints = []
# 	scale = 1.0
		
# 	wpose = arm_group.get_current_pose().pose
# 	wpose.position.z -= scale * 0.1  # First move up (z)
# 	wpose.position.y += scale * 0.2  # and sideways (y)
# 	waypoints.append(copy.deepcopy(wpose))
	
# 	wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
# 	waypoints.append(copy.deepcopy(wpose))
	
# 	wpose.position.y -= scale * 0.1  # Third move sideways (y)
# 	waypoints.append(copy.deepcopy(wpose))
	
# 	(plan, fraction) = arm_group.compute_cartesian_path(
#                                    waypoints,   # waypoints to follow
#                                    0.01,        # eef_step
#                                    0.0)         # jump_threshold

# 	## Now, we call the planner to compute the plan and execute it.
# 	arm_group.execute(plan, wait=True)
# 	arm_group.stop()
# 	# It is always good to clear your targets after planning with poses.
# 	# Note: there is no equivalent function for clear_joint_value_targets()
# 	arm_group.clear_pose_targets()

# 	## END_SUB_TUTORIAL

# 	# For testing:
# 	# Note that since this section of code will not be included in the tutorials
# 	# we use the class variable rather than the copied state variable
# 	current_pose = arm_group.get_current_pose().pose
# 	print "EE_POSE", EE_POSE
	

# def cartesian_control_with_IMU(arm_group, robot_init, hand_pose, *argv):
# 	waypoints = []
# 	scale = 1.0
	
# 	wpose = Pose()
# 	wpose.position.x = robot_init.position.x + scale * hand_pose.position.y
# 	wpose.position.y = robot_init.position.y + scale * hand_pose.position.z
# 	wpose.position.z = robot_init.position.z + scale * hand_pose.position.x
# 	wpose.orientation = robot_init.orientation
# 	# wpose.orientation = kinematic.q_multiply(robot_init.orientation, hand_pose.orientation)
	
# 	waypoints.append(copy.deepcopy(wpose))
	
# 	(plan, fraction) = arm_group.compute_cartesian_path(
#                                    waypoints,   # waypoints to follow
#                                    0.01,        # eef_step
#                                    0.0)         # jump_threshold

# 	arm_group.execute(plan, wait=True)
# 	arm_group.stop()
# 	arm_group.clear_pose_targets()


# def odometryCb_tool0(msg):
# 	global EE_POSE, t
# 	'''
# 	msg: world to wrist_3_link
# 	EE_POSE should be calculated here according to: 
	
# 	rosrun tf tf_echo /tool0 /wrist_3_link
# 	At time 0.000
# 	- Translation: [0.000, -0.000, -0.082]
# 	- Rotation: in Quaternion [0.707, -0.000, -0.000, 0.707]
# 							in RPY (radian) [1.571, -0.000, -0.000]
# 							in RPY (degree) [90.000, -0.000, -0.000]
# 	'''

# 	EE_POSE = msg
# 	# print msg.pose.pose
	    

# def main():
#     global t
    
#     # create /tf wrist_3_link to /tool0
#     t.header.stamp = rospy.Time.now()
#     t.header.frame_id = "wrist_3_link"
#     t.child_frame_id = "tool0"
#     t.transform.translation.x = 0.0
#     t.transform.translation.y = 0.0
#     t.transform.translation.z = -0.082
#     t.transform.rotation.x = 0.707
#     t.transform.rotation.y = -0.000
#     t.transform.rotation.z = -0.000
#     t.transform.rotation.w = 0.707
    
#     try:
# 		arm_group = movegroup_init()		
# 		# rospy.Subscriber('/odom_tool0',Odometry,odometryCb_tool0)
# 		# rospy.sleep(5)

# 		IMU.init_subscribers_and_publishers()

# 		robot_init = arm_group.get_current_pose().pose
# 		print "============ Arm current pose: ", robot_init
# 		print "click Enter to continue"
# 		dummy_input = raw_input()
# 		prev = time.time()
# 		while not rospy.is_shutdown():
# 			if IMU.calibration_flag < 21:
# 				print "calibration:", IMU.calibration_flag
# 			else:
# 				# robot_init = Pose(Point(-0.175, 0.000, -0.095), Quaternion(0.000, 0.000, -0.707, 0.707))				print "robot_init:", robot_init
# 				IMU.hand_pos_calculate()
# 				GOAL_POSE = IMU.tf_wrist
# 				print "GOAL_POSE", GOAL_POSE
# 				cartesian_control_with_IMU(arm_group, robot_init, GOAL_POSE)
# 				# wpose = arm_group.get_current_pose().pose
# 				# print "wpose:", wpose
# 				# print "Enter x_val"
# 				# x_val = float(raw_input())
# 				# print "Enter y_val"
# 				# y_val = float(raw_input())
# 				# print "Enter z_val"
# 				# z_val = float(raw_input())
# 				# cartesian_control_with_IMU(arm_group, robot_init, GOAL_POSE, x_val, y_val, z_val)
# 				# task_space_control(arm_group, x_val, y_val, z_val)
# 			IMU.update()
# 			IMU.r.sleep()
			
			


#     except KeyboardInterrupt:
#         moveit_commander.roscpp_shutdown()
#         rospy.signal_shutdown("KeyboardInterrupt")
#         raise


if __name__ == '__main__': 
    # main()
    print("here")





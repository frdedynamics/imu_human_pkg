#!/usr/bin/env python3

"""
Human-robot-task merge happens here

(Whole refactored robot_move_node. Human and Robot commanders are seperated and the code is cleaned)
"""

import rospy,sys,numpy, time
from Classes.robot_commander_class import RobotCommander
from Classes.human_commander_class import HumanCommander
from Classes.colift_thread_class import ForceThread

from geometry_msgs.msg import Quaternion

# TODO:
# from Classes.task_environment_class import TaskEnvironment
'''
	Arduino buttons subscriber, table imu subscriber. All will be in TaskEnvironment class.
'''


def main(): 
	Robot = RobotCommander(start_node=False)
	Human = HumanCommander(start_node=False)
	# Task = TaskEnvironment(start_node=False)
	rospy.init_node("HRC_state_machine_node")
	print("robot_move_with_ur_rtde Node Created")
	Robot.init_subscribers_and_publishers()
	Human.init_subscribers_and_publishers()

	## Required initializations
	Human.human_to_robot_init_orientation = Quaternion(0.0, 0.0, 0.707, 0.707)
	Robot.rtde_c.moveL(Robot.robot_init_list)
	if not Robot.open_gripper():
		Exception("Please activate gripper")
	rate = rospy.Rate(100)
	try:
		while not rospy.is_shutdown():
			Robot.update()
			Human.update()
			# Task.update()
			hrc_state, state_transition_flag = Human.get_state()
			state_machine(Human, Robot, hrc_state, state_transition_flag)
			rate.sleep()
	except KeyboardInterrupt:
		rospy.signal_shutdown("KeyboardInterrupt")
		raise

def state_machine(human_commander, robot_commander, state, state_transition_flag):

	## TODO: Add teleop active and teleop idle later

	if state == "IDLE":
		robot_commander.rtde_c.servoStop()
		robot_commander.rtde_c.forceModeStop()
		if state_transition_flag:
			human_commander.hands_reset()
			# resets hands origin everytime
			## To connect with the HRC training button, maybe rosparam needed from the init node
	
	elif state == "APPROACH":
		robot_commander.rtde_c.forceModeStop()
		if state_transition_flag:
			robot_commander.get_approach_init_TCP_pose()
			# hope to eliminate the jumps

		robot_goal_pose = human_commander.two_hands_move(robot_commander.approach_init_TCP_pose)
		robot_commander.move_relative_to_current_pose(robot_goal_pose)

	elif state == "COLIFT":
		robot_commander.rtde_c.servoStop()
		if state_transition_flag:
			robot_commander.close_gripper()
			rospy.sleep(2)
		robot_commander.get_colift_init_TCP_pose()
		dir_str, dir_change_flag = human_commander.get_dir_from_elbows()
		force_thread = ForceThread(rtde_r=robot_commander.rtde_r, rtde_c=robot_commander.rtde_c, mode=dir_str)
		force_thread.join() # this blocks until the process terminates

		if dir_change_flag:
			print("flag true")
			robot_commander.forceModeStop()
			if force_thread.is_alive():
				Exception("ForceThread still alive")
			force_thread = ForceThread(rtde_r=robot_commander.rtde_r, rtde_c=robot_commander.rtde_c, mode=dir_str)
			force_thread.join()
			dir_change_flag = False
			# prev = time.time()
	
	elif state == "RELEASE":
		robot_commander.rtde_c.servoStop()
		robot_commander.rtde_c.forceModeStop()

		print("Moving to RELEASE pose")
		robot_commander.rtde_c.moveL(robot_commander.release_before)
		robot_commander.rtde_c.moveL(robot_commander.release)
		robot_commander.open_gripper()
		print("Robot at RELEASE")
		rospy.sleep(4)  # Wait until the gripper is fully open
		robot_commander.rtde_c.moveL(robot_commander.release_after)
		print("Robot at RELEASE APPROACH")
		## new cycle 
		user_input = input("Ready to new cycle?")
		if user_input == 'y':
			robot_commander.rtde_c.moveJ(robot_commander.home_teleop_approach_joints, speed=0.5)
			robot_commander.rtde_c.moveJ(robot_commander.home_teleop_joints, speed=0.5)
			robot_commander.colift_dir = 'up'
	else:
		print("state:", state)
		Exception("Unknown state. Exiting...")


if __name__ == '__main__': main()
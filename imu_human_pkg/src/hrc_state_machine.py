#!/usr/bin/env python3

"""
Whole refactored robot_move_node. Human and Robot commanders are seperated and the code is cleaned
"""

import rospy,sys,numpy, time
from Classes.robot_commander_class import RobotCommander
from Classes.human_commander_class import HumanCommander
from Classes.colift_thread_class import ForceThread

# TODO:
# from Classes.task_environment_class import TaskEnvironment
'''
	Arduino buttons subscriber, table imu subscriber. All will be in TaskEnvironment class.
'''


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

		human_commander.calculate_relative_merged_hands(robot_commander.get_current_TCP_pose())
		robot_commander.move_relative_to_current_pose(human_commander.merged_hands)

	elif state == "COLIFT":
		robot_commander.close_gripper() # dont forget rospy.sleep(2) to make sure proper close
		robot_commander.rtde_c.servoStop()
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


def main(): 
	Robot = RobotCommander(start_node=False)
	Human = HumanCommander(start_node=False)
	# Task = TaskEnvironment(start_node=False)
	rospy.init_node("HRC_state_machine_node")
	print("robot_move_with_ur_rtde Node Created")
	Robot.init_subscribers_and_publishers()
	Human.init_subscribers_and_publishers()
	Robot.move_pose(Robot.home_pose)
	prev_hrc_state = ""
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


if __name__ == '__main__': main()
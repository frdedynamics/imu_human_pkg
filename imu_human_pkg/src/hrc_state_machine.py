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
	# Robot.rtde_c.moveL(Robot.init_list)
	if not Robot.open_gripper():
		Exception("Please activate gripper")
	rate = rospy.Rate(100)
	try:
		while not rospy.is_shutdown():
			Robot.update()
			Human.update()
			# Task.update()
			hrc_state, state_transition_flag = Human.get_state()
			state_machine(Human, Robot, hrc_state.data, state_transition_flag)
			rate.sleep()
	except KeyboardInterrupt:
		rospy.signal_shutdown("KeyboardInterrupt")
		raise

def state_machine(human_commander, robot_commander, state, state_transition_flag):

	## TODO: Add teleop active and teleop idle later

	print("state: ", state)

	if state == "IDLE":
		robot_commander.rtde_c.servoStop()
		robot_commander.rtde_c.forceModeStop()
		if state_transition_flag:
			human_commander.hands_reset()
			print("Human hands reset.")
			# resets hands origin everytime
	
	
	elif state == "APPROACH":
		if state_transition_flag:
			robot_commander.set_approach_init_TCP_pose()
			robot_commander.rtde_c.forceModeStop()
			# hope to eliminate the jumps
		robot_goal_pose = human_commander.two_hands_move(robot_commander.approach_init_TCP_list)
		robot_commander.move_relative_to_current_pose(robot_goal_pose)


	elif state == "COLIFT":
		robot_commander.rtde_c.servoStop()
		if state_transition_flag:
			robot_commander.close_gripper()
			rospy.sleep(2)
			robot_commander.set_colift_init_TCP_pose()
			force_thread = ForceThread(rtde_r=robot_commander.rtde_r, rtde_c=robot_commander.rtde_c, mode="up")
			force_thread.join()
			while force_thread.is_alive():
				print("wait for compliance mode to be ready")
		
		_curr_force = robot_commander.rtde_r.getActualTCPForce()
		if abs(_curr_force[0]) > 1.6:
			dir_str, dir_change_flag = human_commander.get_dir_from_elbows()
			force_thread = ForceThread(rtde_r=robot_commander.rtde_r, rtde_c=robot_commander.rtde_c, mode=dir_str)
			force_thread.join()

			if dir_change_flag:
				print("flag true")
				robot_commander.rtde_c.forceModeStop()
				if force_thread.is_alive():
					Exception("ForceThread still alive")
				force_thread = ForceThread(rtde_r=robot_commander.rtde_r, rtde_c=robot_commander.rtde_c, mode=dir_str)
				force_thread.join()
				dir_change_flag = False

	
	elif state == "RELEASE":
		print("HERE RELEASE")
		robot_commander.rtde_c.servoStop()
		robot_commander.rtde_c.forceModeStop()
		print("Moving to RELEASE pose")
		release_pose = robot_commander.rtde_c.getForwardKinematics(robot_commander.release_joints, robot_commander.tcp_offset)
		before_release_pose = release_pose
		release_pose[2] += 0.3
		robot_commander.rtde_c.moveL(before_release_pose)
		release_pose = robot_commander.rtde_c.getForwardKinematics(robot_commander.release_joints, robot_commander.tcp_offset)
		robot_commander.rtde_c.moveL(release_pose)
		robot_commander.open_gripper()
		print("Robot at RELEASE")
		after_release_pose = release_pose
		after_release_pose[1] += -0.1
		robot_commander.rtde_c.moveL(after_release_pose)
		print("Robot at RELEASE APPROACH")
		## new cycle 
		user_input = input("Ready to new cycle?")
		if user_input == 'y':
			robot_commander.rtde_c.moveJ(robot_commander.before_home_joints, speed=1.0)
			robot_commander.rtde_c.moveJ(robot_commander.init_joints, speed=1.0)
			robot_commander.colift_dir = 'up'
			input("Press a key when the user is ready in IDLE state")
		else:
			rospy.signal_shutdown("KeyboardInterrupt")
			sys.exit()
	else:
		print("state:", state)
		Exception("Unknown state. Exiting...")


if __name__ == '__main__': main()
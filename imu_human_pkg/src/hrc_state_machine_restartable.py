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
from std_msgs.msg import Bool, Float32MultiArray

from subprocess import Popen

# TODO:
# from Classes.task_environment_class import TaskEnvironment
'''
	Arduino buttons subscriber, table imu subscriber. All will be in TaskEnvironment class.
'''

tcp_force = Float32MultiArray()
colift_flag = False

def robot_tcp_force_cb(msg):
	"""This is a standalone callback to create an interrupt on COLIFT state"""
	global tcp_force, colift_flag
	tcp_force = msg

	# rospy.sleep(0.5) # trying to eliminate double force readins
	if abs(tcp_force.data[1]) > 20:
		colift_flag = True


def main(): 
	Robot = RobotCommander(start_node=False)
	Human = HumanCommander(start_node=False)
	# Task = TaskEnvironment(start_node=False)
	rospy.init_node("HRC_state_machine_node")
	print("HRC_state_machine_node Node Created")
	Robot.init_subscribers_and_publishers()
	Human.init_subscribers_and_publishers()


	game_over_flag = Bool()
	game_over_flag.data = False

	pub_game_over_flag = rospy.Publisher("/game_over_flag", Bool, queue_size=1)

	## Required initializations
	Human.human_to_robot_init_orientation = Quaternion(0.0, 0.0, 0.707, 0.707)
	Robot.rtde_c.moveJ(Robot.home_joints)
	if not Robot.open_gripper():
		Exception("Please activate gripper")
	rate = rospy.Rate(100)
	state_transition_flag = True
	rospy.set_param('/robot_move_started', True)
	try:
		while not rospy.is_shutdown():
			Robot.update()
			Human.update()
			# Task.update()
			hrc_state, state_transition_flag = Human.get_state()
			state_machine(Human, Robot, hrc_state.data, state_transition_flag, game_over_flag)
			pub_game_over_flag.publish(game_over_flag)
			rate.sleep()

			if game_over_flag.data == True:
			#restart_new_trial()
				user_input = input("Ready to new cycle?")
				if user_input == 'y':
					rospy.set_param('/robot_move_started', False)
					rospy.set_param('/colift_set', False)
					Robot.rtde_c.disconnect()
					# Popen(["rosnode", "kill", "/visualize_and_gamify"])
					print("Please stop the robot and start from measure thresholds")
					print("REMEMBER ARDUINO BUTTONS!!!")
					state = "IDLE"
					game_over_flag.data = False
	except KeyboardInterrupt:
		Robot.rtde_c.forceModeStop()
		rospy.signal_shutdown("KeyboardInterrupt")
		raise

def state_machine(human_commander, robot_commander, state, state_transition_flag, game_over_flag):

	## TODO: Add teleop active and teleop idle later

	global tcp_force, colift_flag

	if state == "IDLE":
		robot_commander.rtde_c.servoStop()
		robot_commander.rtde_c.forceModeStop()
	
	
	elif state == "APPROACH":
		if state_transition_flag:
			robot_commander.set_approach_init_TCP_pose()
			robot_commander.rtde_c.forceModeStop()
			human_commander.hands_reset()
		robot_goal_pose = human_commander.two_hands_move(robot_commander.approach_init_TCP_list)
		robot_commander.move_relative_to_current_pose(robot_goal_pose, lookahead_time=robot_commander.lookahead_time)


	elif state == "COLIFT":
		robot_commander.rtde_c.servoStop()
		if state_transition_flag:
			robot_commander.close_gripper()
			print("Closing gripper")
			robot_commander.set_colift_init_TCP_pose()
			force_thread = ForceThread(rtde_r=robot_commander.rtde_r, rtde_c=robot_commander.rtde_c, mode="u", force=robot_commander.colift_force)
			force_thread.join()
			while force_thread.is_alive():
				print("wait for compliance mode to be ready")
		
		if colift_flag:  ## TODO: make it as interrupt
			print("force: ",(tcp_force.data[1]))
			dir_str, dir_change_flag = human_commander.get_dir_from_elbows(tcp_force.data[1])
			force_thread = ForceThread(rtde_r=robot_commander.rtde_r, rtde_c=robot_commander.rtde_c, mode=dir_str, force=robot_commander.colift_force)
			force_thread.join()
			colift_flag = False

			if dir_change_flag:
				print("flag true")
				robot_commander.rtde_c.forceModeStop()
				if force_thread.is_alive():
					Exception("ForceThread still alive")
				force_thread = ForceThread(rtde_r=robot_commander.rtde_r, rtde_c=robot_commander.rtde_c, mode=dir_str, force=robot_commander.colift_force)
				force_thread.join()
				dir_change_flag = False

	
	elif state == "RELEASE":
		if state_transition_flag == True:
			print("HERE RELEASE")
			robot_commander.rtde_c.servoStop()
			robot_commander.rtde_c.forceModeStop()
			robot_commander.close_gripper()
			print("Moving to RELEASE pose")

		if robot_commander.rtde_c.isSteady():
			print("Robot steady")
			if len(robot_commander.target_poses) == 0:
				print("THE END")
				game_over_flag.data = True
				# robot_commander.rtde_c.disconnect()
				# sys.exit()
			elif len(robot_commander.target_poses) == 1:
				robot_commander.open_gripper()
				print("gripper open")
				robot_commander.rtde_c.moveL(robot_commander.target_poses[0], robot_commander.release_speed, 1.2, True)
				robot_commander.target_poses.pop(0)
			else:
				print(robot_commander.target_poses)
				robot_commander.rtde_c.moveL(robot_commander.target_poses[0], robot_commander.release_speed, 1.2, True)
				robot_commander.target_poses.pop(0)
			rospy.sleep(0.5)
		else:
			pass


		# if robot_commander.rtde_c.isSteady():
		# 	print("Robot steady")
		# 	if len(target_poses) == 0:
		# 		print("THE END")
		# 		game_over_flag.data = True
		# 		robot_commander.rtde_c.servoStop()

		# 		##restart_new_trial()
		# 		user_input = input("Ready to new cycle?")
		# 		if user_input == 'y':
		# 			if rospy.has_param('/robot_move_started'):
		# 				rospy.delete_param('/robot_move_started')
		# 			if rospy.has_param('/colift_set'):
		# 				rospy.delete_param('/colift_set')
		# 			if rospy.has_param('/elbow_height_th'):
		# 				rospy.delete_param('/elbow_height_th')
		# 			if rospy.has_param('/emg_sum_th'):
		# 				rospy.delete_param('/emg_sum_th')
		# 			Popen(["rosnode", "kill", "/visualize_and_gamify"])
		# 			state = "IDLE"
		# 			game_over_flag.data = False

		# 			sys.exit()
		# 	elif len(target_poses) == 1:
		# 		robot_commander.open_gripper()
		# 		print("gripper open")
		# 	else:
		# 		print("here")
		# 		print(target_poses)
		# 		robot_commander.rtde_c.moveL(target_poses[0], 0.25, 1.2, True)
		# 		target_poses.pop(0)
		# 	rospy.sleep(0.5)
		

		
			
		# else:
		# 	sys.exit()


		## Fix it later:
		# Popen(["rosnode", "kill", "/rviz_markers"])
		# sys.exit()
		## new cycle 
		# user_input = input("Ready to new cycle?")
		# if user_input == 'y':
		# 	robot_commander.rtde_c.moveJ(robot_commander.before_home_joints, speed=1.0)
		# 	robot_commander.rtde_c.moveJ(robot_commander.init_joints, speed=1.0)
		# 	robot_commander.colift_dir = 'up'
		# 	input("Press a key when the user is ready in IDLE state")
		# else:
		# 	rospy.signal_shutdown("KeyboardInterrupt")
		# 	sys.exit()
	else:
		print("state:", state)
		raise Exception("Unknown state. Exiting...")


if __name__ == '__main__': main()
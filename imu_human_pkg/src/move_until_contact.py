#! /usr/bin/env python3

"""
Trying thread
"""

from numpy.core.numeric import count_nonzero
import rospy
from os import stat
import threading
import time
from math import radians as d2r

from Classes.robot_move_with_ur_rtde_with_TO import RobotCommander

# def thread_function():
#     _result = robot.rtde_c.moveUntilContact([0, 0, -0.03, 0, 0, 0], [0, 0, 1, 0, 0, 0])

if __name__ == "__main__":
    robot = RobotCommander(start_node=True)
    # TODO: any problem coming from IDLE but not from APPROACH?
    ''' Make force thingy here '''
    # vector = robot.rtde_r.getActualTCPPose() # A pose vector that defines the force frame relative to the base frame.
    vector = [0, 0, 0, 0, 0, 0]
    selection_vector = [1, 1, 1, 0, 0, 0] # A 6d vector of 0s and 1s. 1 means that the robot will be compliant in the corresponding axis of the task frame
    wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # The forces/torques the robot will apply to its environment. The robot adjusts its position along/about compliant axis in order to achieve the specified force/torque. Values have no effect for non-compliant axes
    type = 2 # An integer [1;3] specifying how the robot interprets the force frame. 1: The force frame is transformed in a way such that its y-axis is aligned with a vector pointing from the robot tcp towards the origin of the force frame. 2: The force frame is not transformed. 3: The force frame is transformed in a way such that its x-axis is the projection of the robot tcp velocity vector onto the x-y plane of the force frame.
    limits = [10, 10, 10, 0.17, 0.17, 0.17]# (Float) 6d vector. For compliant axes, these values are the maximum allowed tcp speed along/about the axis. For non-compliant axes, these values are the maximum allowed deviation along/about an axis between the actual tcp position and the one set by the program.

    robot.colift_dir = 'up'
    robot.colift_flag = False

    while not rospy.is_shutdown():
        _curr_force = robot.rtde_r.getActualTCPForce()
        # print(_curr_force[0])

        if robot.colift_dir == 'right':
            # print(robot.status)
            # vector = robot.rtde_r.getActualTCPPose()
            selection_vector = [1, 1, 1, 0, 0, 0] 
            wrench = [10.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            limits = [0.5, 0.3, 0.3, 0.17, 0.17, 0.17]

        elif robot.colift_dir == 'left':
            # print(robot.status)
            # vector = robot.rtde_r.getActualTCPPose()
            selection_vector = [1, 1, 1, 0, 0, 0]
            wrench = [-10.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            limits = [0.5, 0.3, 0.3, 0.17, 0.17, 0.17]

        elif robot.colift_dir == 'up':
            # print(robot.status)
            # vector = robot.rtde_r.getActualTCPPose()
            selection_vector = [1, 1, 1, 0, 0, 0]
            wrench = [0.0, 0.0, 10.0, 0.0, 0.0, 0.0]
            limits = [0.3, 0.3, 0.5, 0.17, 0.17, 0.17]

        print(_curr_force[0])
        
        if abs(_curr_force[0]) > 1:
            # print(_curr_force[0])
            # print("Side movement")
            # print(robot.elbow_right_height, robot.elbow_left_height)
            height_th = 0.1
            # colift_dir = ''
            # colift_dir_past = ''
            if((robot.elbow_right_height > height_th) and (robot.elbow_left_height < height_th)):
                robot.colift_dir = 'right'
            elif((robot.elbow_left_height > height_th) and (robot.elbow_right_height < height_th)):
                robot.colift_dir = 'left'
            else:
                if not robot.colift_flag:
                    robot.colift_flag = True
                    print(_curr_force[0])
                else:
                    robot.colift_dir = 'null'

        # print(robot.colift_dir)
        robot.rtde_c.forceMode(vector, selection_vector, wrench, type, limits)


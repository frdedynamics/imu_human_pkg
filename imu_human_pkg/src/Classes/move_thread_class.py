#! /usr/bin/env python3

"""
This is just a python script -NO ROS- for subprocess to test COLIFT state
"""

import threading
from sys import argv, stdout


class MoveThread(threading.Thread):
    def __init__(self, rtde_r, rtde_c, goal, group=None, deamon=True):
        threading.Thread.__init__(self)
        self.rtde_r = rtde_r
        self.rtde_c = rtde_c
        self.goal = goal
        print("Moving to: ", self.goal)
        self.start()
        print("thread created!")

    
    def run(self):
        print("thread run!")
        self.rtde_c.moveL(self.goal)
        print("Applied force: ", self.goal)


        # _curr_force = self.rtde_r.getActualTCPForce()
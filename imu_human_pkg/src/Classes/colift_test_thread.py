#! /usr/bin/env python3

"""
This is just a python script -NO ROS- for subprocess to test COLIFT state
"""

import threading
from sys import argv, stdout


class ForceThread(threading.Thread):
    def __init__(self, rtde_r, mode, group=None, deamon=True):
        threading.Thread.__init__(self)
        # super(ForceThread, self).__init__(self) ## ERROR: assert group is None, "group argument must be None for now"
        self.rtde_r = rtde_r
        self.mode = mode
        self.start()
        print("thread creates!")


    ## I need super().__init__ to override stop event.
    # def stop(self):
    #     self._stop_event.set()


    # def stopped(self):
    #     return self._stop_event.is_set()

    
    def run(self):
        print("thread here!")
        f=15
        vector = [0, 0, 0, 0, 0, 0]
        type = 2 
        selection_vector = [1, 0, 0, 0, 0, 0]
        # wrench = [-f, 0.0, 0.0, 0.0, 0.0, 0.0]
        limits = [0.5, 0.3, 0.3, 0.17, 0.17, 0.17]

        if self.mode =='l':
            wrench = [f, 0.0, 0.0, 0.0, 0.0, 0.0]
            # rtde_c.forceMode(vector, selection_vector, wrench, type, limits)
        elif self.mode =='r':
            wrench = [-f, 0.0, 0.0, 0.0, 0.0, 0.0]
            # rtde_c.forceMode(vector, selection_vector, wrench, type, limits)
        else:
            wrench = [0, 0.0, 0.0, 0.0, 0.0, 0.0]
            # self.rtde_c.forceModeStop()

        print("thread here!")
        # _curr_force = self.rtde_r.getActualTCPForce()
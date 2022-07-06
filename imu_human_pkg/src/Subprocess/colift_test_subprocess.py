#! /usr/bin/env python3

"""
This is just a python script -NO ROS- for subprocess to test COLIFT state
"""

import subprocess
from sys import argv, stdout

# must be a global function    
def force_apply(f, mode, rtde_c):
    vector = [0, 0, 0, 0, 0, 0]
    type = 2 
    selection_vector = [1, 0, 0, 0, 0, 0]
    # wrench = [-f, 0.0, 0.0, 0.0, 0.0, 0.0]
    limits = [0.5, 0.3, 0.3, 0.17, 0.17, 0.17]

    if mode =='l':
        wrench = [f, 0.0, 0.0, 0.0, 0.0, 0.0]
        # rtde_c.forceMode(vector, selection_vector, wrench, type, limits)
    elif mode =='r':
        wrench = [-f, 0.0, 0.0, 0.0, 0.0, 0.0]
        # rtde_c.forceMode(vector, selection_vector, wrench, type, limits)
    else:
        wrench = [0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # rtde_c.forceModeStop()


if __name__ == '__main__':
    # force_apply(argv[1], argv[2], argv[3])
    # print(argv[1], argv[2], argv[3])
    stdout.write('asd')
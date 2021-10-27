#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
The files should be layouted as following:
    - All generated CSV files should be placed somewhere within the 'Output' folder
    - Every set of CSV files that belong to one Tester should be placed in a separate subfolder
    for example, if a user Ahmet ran the simulation 10 times, then 10 CSV files were generated,
    we place all of them in a folder called 'Ahmet' or some random number, it doesn't really matter.
    - Basically every subfolder represents one set of training runs. One person yani
    - The generated CSV files should be named with their timestamped
    so that there filenames are can be sorted in increasing order.
#todo: deal with different simulation scenarios (with vs without axis alignment)
"""

import os
from datetime import datetime


# OUTPUT_FOLDER = "../Output"  # take it as a rosparam
# OUTPUT_FOLDER = "/home/gizem/Documents/Gitkraken/arm-paper/DataOutput"  # take it as a rosparam
OUTPUT_FOLDER = "/home/gizem/catkin_ws/src/arm_motion_controller_py3/logging/Data/Output"  # take it as a rosparam
OUTPUT_FILENAME_PREFIX = "HRI"



# DATA_LABELS = ('ID_ELAPSED_TIME',
# 				'ID_LHAND_POS_X',
# 				'ID_LHAND_POS_Y',
# 				'ID_LHAND_POS_Z',
# 				'ID_LHAND_ORI_X',
# 				'ID_LHAND_ORI_Y',
# 				'ID_LHAND_ORI_Z',
# 				'ID_LHAND_ORI_W',
# 				'ID_RHAND_POS_X',
# 				'ID_RHAND_POS_Y',
# 				'ID_RHAND_POS_Z',
# 				'ID_RHAND_ORI_X',
# 				'ID_RHAND_ORI_Y',
# 				'ID_RHAND_ORI_Z',
# 				'ID_RHAND_ORI_W',
# 				'ID_HAND_POS_X',
# 				'ID_HAND_POS_Y',
# 				'ID_HAND_POS_Z',
# 				'ID_HAND_ORI_X',
# 				'ID_HAND_ORI_Y',
# 				'ID_HAND_ORI_Z',
# 				'ID_HAND_ORI_W',
# 				'ID_TGOAL_POS_X',
# 				'ID_TGOAL_POS_Y',
# 				'ID_TGOAL_POS_Z',
# 				'ID_TGOAL_ORI_X',
# 				'ID_TGOAL_ORI_Y',
# 				'ID_TGOAL_ORI_Z',
# 				'ID_TGOAL_ORI_W',
# 				'ID_TACTUAL_POS_X',
# 				'ID_TACTUAL_POS_Y',
# 				'ID_TACTUAL_POS_Z',
# 				'ID_TACTUAL_ORI_X',
# 				'ID_TACTUAL_ORI_Y',
# 				'ID_TACTUAL_ORI_Z',
# 				'ID_TACTUAL_ORI_W')

DATA_LABELS = ('ID_ELAPSED_TIME',
				'ID_LHAND',
				'ID_RHAND',
				'ID_HAND',
                'ID_LELBOW',
                'ID_RELBOW',
                'ID_GRIP_STRENGTH',
                'ID_TCP',
                'ID_STATUS')

DATA_INDICES = list(range(0,len(DATA_LABELS)))


def get_new_filename(user_name, ref):
    postfix = ref + '_' + user_name + '_' + datetime.now().strftime('%Y-%m-%d-%H-%M-%S') +".csv"
    filename = OUTPUT_FILENAME_PREFIX + "_" + postfix
    # filename = "full_cycle_rt.csv"
    # if folder doesn't exist, create it
    if not os.path.exists(OUTPUT_FOLDER):
        os.makedirs(OUTPUT_FOLDER)
    filename = os.path.join(OUTPUT_FOLDER, filename)
    return filename
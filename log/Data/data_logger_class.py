#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This is a data_logger class. Subscribes the IMU readings and save them into a CSV fileself.

"""

# imports
from queue import Queue, Empty
import threading
import csv
import Data.data as data
import rospy


# Private Globals
_DATA = Queue()
_DATA_LOGGER = None
_logging_enabled = False
_ref_frame = None


# Functions
# def log_metrics(time, tee, human_wrist_angles, hand_pose, gain):
def log_metrics(time, lhand_pose, rhand_pose, hand_pose, lelbow_height, relbow_height, grip_strength, tcp_current, status):
    if _DATA_LOGGER.running:
        lhand_pose_arr = [lhand_pose.position.x, lhand_pose.position.y, lhand_pose.position.z, lhand_pose.orientation.x, lhand_pose.orientation.y, lhand_pose.orientation.z, lhand_pose.orientation.w]
        rhand_pose_arr = [rhand_pose.position.x, rhand_pose.position.y, rhand_pose.position.z, rhand_pose.orientation.x, rhand_pose.orientation.y, rhand_pose.orientation.z, rhand_pose.orientation.w]
        hand_pose_arr = [hand_pose.position.x, hand_pose.position.y, hand_pose.position.z, hand_pose.orientation.x, hand_pose.orientation.y, hand_pose.orientation.z, hand_pose.orientation.w]
        
        new_data = [time, lhand_pose_arr, rhand_pose_arr, hand_pose_arr, lelbow_height, relbow_height, grip_strength, tcp_current, status]
        _DATA.put(new_data)
        # print new_data
    else:
        return


def enable_logging(username, ref):
    global _DATA_LOGGER
    global _logging_enabled
    _logging_enabled = True
    # username = 'gizem'
    # ref = 'base'
    _DATA_LOGGER = DataLogger(username, ref)  # thread here
    _DATA_LOGGER.start() ## start thread not start() module of your logger.
    print("enable_logging")


def disable_logging():
    global _logging_enabled
    if _logging_enabled:
        if _DATA_LOGGER.running:
            _DATA_LOGGER.stop()
        _logging_enabled = False
        print("disable_logging")


class DataLogger(threading.Thread):
    def __init__(self, username, ref):
        self.username = username
        self.ref = ref
        threading.Thread.__init__(self)
        self.daemon = True
        self.filename = data.get_new_filename(self.username, self.ref)
        self.fp = open(self.filename, 'w')
        self.writer = csv.writer(self.fp, lineterminator='\n')
        # write the header of the CSV file (the labels of each field/feature)
        print("labels:", data.DATA_LABELS)
        self.writer.writerow(data.DATA_LABELS)
        self.running = True

    def run(self):
        while self.running:
            try:
                row = _DATA.get(timeout=1)
                print("data:", row)
                self.writer.writerow(row)
            except Empty:
                continue
        self.close()

    def close(self):
        if self.fp is not None:
            self.fp.close()
            self.fp = None

    def stop(self):
        self.running = False

    def __del__(self):
        self.close()
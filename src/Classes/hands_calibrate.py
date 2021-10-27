#! /usr/bin/env python3

"""
Subscrives two wrist poses w.r.t chest. Send Tee goal w.r.t UR5e initial pose at home.

This class is a newer version of wrist_tp_robot_2arms.py
Action server is included because I need to initialize hand pose in Colift state.
"""

import numpy as np
from Classes import Kinematics_with_Quaternions as kinematics

import rospy, actionlib
from arm_motion_controller_py3.msg import handCalibrationAction, handCalibrationFeedback, handCalibrationResult
from geometry_msgs.msg import Pose, Point, Quaternion

from Classes.DH_matrices import DHmatrices

DHmatrices = DHmatrices()

class HandCalibrate:
    def __init__(self):
        self.wrist_left_pose = Pose()
        self.wrist_right_pose = Pose()
        self.init_flag = False
        self.wrist_left_ori_init = Quaternion()
        self.wrist_right_ori_init = Quaternion()
        self.left = Pose()
        self.right = Pose()
        print("Initiated")

    def init_node(self, rate=100.0):
        self.pub_left_hand_pose = rospy.Publisher('/left_hand_pose', Pose, queue_size=1)
        self.pub_right_hand_pose = rospy.Publisher('/right_hand_pose', Pose, queue_size=1)
        self.sub_l_wrist = rospy.Subscriber('/wrist_left', Pose, self.cb_l_wrist)
        self.sub_r_wrist = rospy.Subscriber('/wrist_right', Pose, self.cb_r_wrist)

        rospy.init_node('wrist_to_robot_2arms')
        print("hands_calibrate node started")
        self.rate = rospy.Rate(rate)

        self.a_server = actionlib.SimpleActionServer(
                "hand_calibration_as", handCalibrationAction, execute_cb=self.as_execute_cb, auto_start=False)
        self.a_server.start()

        
        print("Move to initial arm poses in 4 seconds...")
        # rospy.sleep(1)
        # print("3 seconds...")
        # rospy.sleep(1)
        # print("2 seconds...")
        # rospy.sleep(1)
        print("1 second...")
        rospy.sleep(1)

        self.calc_inv()

    def calc_inv(self):
        if not self.init_flag:
            self.wrist_left_ori_init = kinematics.q_invert(self.wrist_left_pose.orientation)
            self.wrist_right_ori_init = kinematics.q_invert(self.wrist_right_pose.orientation)
            self.init_flag = True
        
        self.left.position = self.wrist_left_pose.position
        self.left.orientation = kinematics.q_multiply(self.wrist_left_ori_init, self.wrist_left_pose.orientation)
        self.right.position = self.wrist_right_pose.position
        self.right.orientation = kinematics.q_multiply(self.wrist_right_ori_init, self.wrist_right_pose.orientation)

        self.left_htm_init = DHmatrices.pose_to_htm(self.left)
        self.right_htm_init = DHmatrices.pose_to_htm(self.right)
        

        self.left_htm_init_inv = np.linalg.inv(self.left_htm_init)
        self.right_htm_init_inv = np.linalg.inv(self.right_htm_init)
        print("Initial arm poses registered")


    def update(self):
        tf_left = np.matmul(self.left_htm_init_inv, DHmatrices.pose_to_htm(self.wrist_left_pose)) 
        tf_right = np.matmul(self.right_htm_init_inv, DHmatrices.pose_to_htm(self.wrist_right_pose))
        self.tf_left_pose = DHmatrices.htm_to_pose(tf_left)
        self.tf_right_pose = DHmatrices.htm_to_pose(tf_right)
        self.pub_left_hand_pose.publish(self.tf_left_pose)
        self.pub_right_hand_pose.publish(self.tf_right_pose)


    def cb_l_wrist(self, msg):
        self.wrist_left_pose = msg

    def cb_r_wrist(self, msg):
        self.wrist_right_pose = msg
    
    def as_execute_cb(self, goal):

        success = True
        calib_flag = True
        feedback = handCalibrationFeedback()
        result = handCalibrationResult()

        if goal.calib_request:
            self.calc_inv()
            feedback.calib_success = calib_flag
            result.left_hand_pose = [self.tf_left_pose.position.x, self.tf_left_pose.position.y, self.tf_left_pose.position.z]
            result.right_hand_pose = [self.tf_right_pose.position.x, self.tf_right_pose.position.y, self.tf_right_pose.position.z]
        else:
            feedback.calib_success = False # not calib_flag
        self.a_server.publish_feedback(feedback)

        if success:
            self.a_server.set_succeeded(result)

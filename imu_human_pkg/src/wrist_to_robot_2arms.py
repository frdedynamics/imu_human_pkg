#! /usr/bin/env python3

"""
Subscrives two wrist poses w.r.t chest. Send Tee goal w.r.t UR5e initial pose at home.

UR5e init \base_link \tool0 TF at initial pose:
- Translation: [-0.136, 0.490, 0.687]
- Rotation: in Quaternion [-0.697, 0.005, 0.012, 0.717]
            in RPY (radian) [-1.542, 0.024, 0.010]
            in RPY (degree) [-88.374, 1.403, 0.549]
"""
import math
import numpy as np

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion

from Classes.DH_matrices import DHmatrices
import Classes.Kinematics_with_Quaternions as kinematic

DHmatrices = DHmatrices()

wrist_left_pose = Pose()
wrist_right_pose = Pose()
ur5e_init = Pose(Point(-0.300613689909, -0.396151355207,  0.171878358801), Quaternion(-0.29428855254, 0.660939846775, 0.650017802692, -0.232442730523))  # home = [pi/2, -pi/2, pi/2, pi, -pi/2, 0.0]

ur5e_init_htm = DHmatrices.pose_to_htm(ur5e_init)


def cb_l_wrist(msg):
    global wrist_left_pose
    wrist_left_pose = msg


def cb_r_wrist(msg):
    global wrist_right_pose
    wrist_right_pose = msg


if __name__ == '__main__':
    pub_motion_hand_pose = rospy.Publisher('/motion_hand_pose', Pose, queue_size=10)
    pub_steering_pose = rospy.Publisher('/steering_hand_pose', Pose, queue_size=10)
    sub_l_wrist = rospy.Subscriber('/wrist_left', Pose, cb_l_wrist)
    sub_r_wrist = rospy.Subscriber('/wrist_right', Pose, cb_r_wrist)
    rospy.init_node('wrist_to_robot_2arms')
    rate = rospy.Rate(100.0)
    print("wrist_to_robot_2arms node started")

    mirror_state = None
    motion_state = None
    init_flag = False

    print("Move to initial arm poses in 4 seconds...")
    rospy.sleep(1)
    print("3 seconds...")
    rospy.sleep(1)
    print("2 seconds...")
    rospy.sleep(1)
    print("1 second...")
    rospy.sleep(1)

    left_htm_init = DHmatrices.pose_to_htm(wrist_left_pose)
    right_htm_init = DHmatrices.pose_to_htm(wrist_right_pose)
    print("Initial arm poses registered")

    left_htm_init_inv = np.linalg.inv(left_htm_init) ## this is such a hard equation for the PC. why?
    right_htm_init_inv = np.linalg.inv(right_htm_init)

    while not rospy.is_shutdown():
        tf_left = np.matmul(left_htm_init_inv, DHmatrices.pose_to_htm(wrist_left_pose)) 
        tf_right = np.matmul(right_htm_init_inv, DHmatrices.pose_to_htm(wrist_right_pose))
        tf_left_pose = DHmatrices.htm_to_pose(tf_left)
        tf_right_pose = DHmatrices.htm_to_pose(tf_right)
        # hand_pose = DHmatrices.htm_to_pose(np.matmul(ur5e_init_htm, tf_left))
        pub_motion_hand_pose.publish(tf_left_pose)
        pub_steering_pose.publish(tf_right_pose)
        # print "tf_left:", tf_left_pose
            
        rate.sleep()


    # while not rospy.is_shutdown():
    #     if not mirror_state == 'y':
    #         print "Move to mirror pose (touch to robot tool). Ready: y"
    #         mirror_state = raw_input()
    #     else:
    #         if not init_flag:
    #             left_htm_init = DHmatrices.pose_to_htm(wrist_left_pose)
    #             right_htm_init = DHmatrices.pose_to_htm(wrist_right_pose)
    #             init_flag = True
    #         if not motion_state == 'y':
    #             print "Start motion? Start: y"
    #             motion_state = raw_input()
    #         else:
    #             tf_left = np.matmul(np.linalg.inv(left_htm_init), DHmatrices.pose_to_htm(wrist_left_pose))
    #             tf_left_pose = DHmatrices.htm_to_pose(tf_left)
    #             # hand_pose = DHmatrices.htm_to_pose(np.matmul(ur5e_init_htm, tf_left))
    #             pub_motion_hand_pose.publish(tf_left_pose)
    #             # print "tf_left:", tf_left_pose
            
    #     rate.sleep()


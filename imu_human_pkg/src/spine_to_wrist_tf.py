#! /usr/bin/env python3

"""
This is a node to publish \wrist_left and \wrist_right transformation with respect to spine fixed frame
"""

import rospy

import math
import tf2_ros
from geometry_msgs.msg import Pose

wrist_left_pose = Pose()
wrist_right_pose = Pose()
elbow_left_pose = Pose()
elbow_right_pose = Pose()

ref = 'human/spine_0'


if __name__ == '__main__':
    pub_left = rospy.Publisher('/wrist_left', Pose, queue_size=10)
    pub_right = rospy.Publisher('/wrist_right', Pose, queue_size=10)
    pub_right_elbow = rospy.Publisher('/elbow_right', Pose, queue_size=10)
    pub_left_elbow = rospy.Publisher('/elbow_left', Pose, queue_size=10)
    rospy.init_node('spine_to_wrist_tf_node')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(100.0)
    rospy.set_param('ref_frame', ref)
    print("Publishing wrist and elbow poses")


    while not rospy.is_shutdown():
        try:
            trans_wrist_left = tfBuffer.lookup_transform(ref, 'human/left_wrist', rospy.Time())
            trans_wrist_right = tfBuffer.lookup_transform(ref, 'human/right_wrist', rospy.Time())
            trans_elbow_left = tfBuffer.lookup_transform(ref, 'human/left_elbow', rospy.Time())
            trans_elbow_right = tfBuffer.lookup_transform(ref, 'human/right_elbow', rospy.Time())

            ## To take TF w.r.t. chest:
            # trans_wrist_left = tfBuffer.lookup_transform('human/shoulder_center', 'human/left_wrist', rospy.Time())
            # trans_wrist_right = tfBuffer.lookup_transform('human/shoulder_center', 'human/right_wrist', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        # print "Translation:", trans.transform.translation
        # print "Rotation:", trans.transform.rotation
        wrist_left_pose.position = trans_wrist_left.transform.translation
        wrist_left_pose.orientation = trans_wrist_left.transform.rotation
        wrist_right_pose.position = trans_wrist_right.transform.translation
        wrist_right_pose.orientation = trans_wrist_right.transform.rotation
        pub_left.publish(wrist_left_pose)
        pub_right.publish(wrist_right_pose)

        elbow_left_pose.position = trans_elbow_left.transform.translation
        elbow_left_pose.orientation = trans_elbow_left.transform.rotation
        elbow_right_pose.position = trans_elbow_right.transform.translation
        elbow_right_pose.orientation = trans_elbow_right.transform.rotation
        pub_left_elbow.publish(elbow_left_pose)
        pub_right_elbow.publish(elbow_right_pose)

        rate.sleep()


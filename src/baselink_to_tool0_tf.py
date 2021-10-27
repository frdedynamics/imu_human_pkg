#! /usr/bin/env python3

"""
This is a node to publish tool0 pose with respect to base link. I only use it if I need to record data.
"""

import rospy

import math
import tf2_ros, tf
from geometry_msgs.msg import Pose, Point, Quaternion

tool0_actual_pose = Pose()
tool0_corrected_pose = Pose()

# rosrun tf tf_echo /wrist_3_link /tool0

# - Translation: [0.000, 0.082, 0.000]
# - Rotation: in Quaternion [-0.707, 0.000, 0.000, 0.707]
#             in RPY (radian) [-1.571, 0.000, 0.000]
#             in RPY (degree) [-90.000, 0.000, 0.000]

if __name__ == '__main__':
    pub_tool = rospy.Publisher('/base_to_tool', Pose, queue_size=10)
    pub_tool_corr = rospy.Publisher('/base_to_tool_corrected', Pose, queue_size=10)
    rospy.init_node('base_to_tool_tf_node')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(10000.0)
    print("Publishing tool actual pose (From robot, not calculated)")

    br = tf.TransformBroadcaster()



    while not rospy.is_shutdown():
        # br.sendTransform((0.000, 0.082, 0.000), (0.707, 0.000, 0.000, 0.707), rospy.Time.now(), 'tool0_corrected', 'wrist_3_link')
        # br.sendTransform((-0.463088628106264, -0.7891308809621151, 0.5275658192110718), (-0.49959937699750445, 0.500250486765663, -0.5001833040508908, -0.4999665742258745), rospy.Time.now(), 'tool0_corrected', 'wrist_3_link')
        # br.sendTransform((-0.6017224641612625, -0.7543159914037476, 0.4081586835170915), (0.4744338572686389, 0.4504965410360891, 0.5674493865082273, 0.49996657422587454), rospy.Time.now(), 'tool0_corrected', 'wrist_3_link')
        br.sendTransform((0.0, -0.04, 0.0), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), 'tool0_corrected', 'wrist_3_link')
        

        try:
            trans_tool0 = tfBuffer.lookup_transform('base_link', 'tool0', rospy.Time())
            trans_tool0_corr = tfBuffer.lookup_transform('base_link', 'tool0_corrected', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        # print "Translation:", trans.transform.translation
        # print "Rotation:", trans.transform.rotation
        tool0_actual_pose.position = trans_tool0.transform.translation
        tool0_actual_pose.orientation = trans_tool0.transform.rotation
        pub_tool.publish(tool0_actual_pose)

        tool0_corrected_pose.position = trans_tool0_corr.transform.translation
        tool0_corrected_pose.orientation = trans_tool0_corr.transform.rotation
        pub_tool_corr.publish(tool0_corrected_pose)

        rate.sleep()


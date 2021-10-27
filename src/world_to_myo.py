#!/usr/bin/env python3
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped, Quaternion


def handle_myo_pose(msg):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "myo_raw"
    t.transform.translation = msg.pose.position
    t.transform.rotation = Quaternion(0.0, 0.0, 0.707, 0.707)

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('tf2_myo_broadcaster')
    rospy.Subscriber('/myo_raw/pose', PoseStamped, handle_myo_pose)
    rospy.spin()
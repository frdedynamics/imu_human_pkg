#! /usr/bin/env python3

"""
This is a node to publish TCP force to create a COLIFT interrupt. Additionally TCP pose publishers moved here for consistency of rtde_receive data publishing from the same node.
"""

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose
import Classes.Kinematics_with_Quaternions as kinematic
import rtde_receive

tcp_force = Float32MultiArray()
current_TCP_list = Float32MultiArray()


def main():
    rtde_r = rtde_receive.RTDEReceiveInterface("172.31.1.144")
    pub_tcp_force = rospy.Publisher('/tcp_force', Float32MultiArray, queue_size=10)
    pub_tcp_current_list = rospy.Publisher('/tcp_pose_list', Float32MultiArray, queue_size=10)
    pub_tcp_current_pose = rospy.Publisher('/tcp_current_pose', Pose, queue_size=10)

    rospy.init_node('rtde_receive_node', anonymous=False)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        tcp_force.data = rtde_r.getActualTCPForce()
        current_TCP_list.data = rtde_r.getActualTCPPose()
        tcp_current_pose = kinematic.list_to_pose(current_TCP_list.data)


        # _curr_force = rtde_r.getActualTCPForce()
        # print(f'0: {_curr_force[0]:.2f} -- 1: {_curr_force[1]:.2f} -- 2: {_curr_force[2]:.2f} -- 3: {_curr_force[3]:.2f} -- 4: {_curr_force[4]:.2f} -- 5: {_curr_force[5]:.2f} -- ')

        pub_tcp_force.publish(tcp_force)
        pub_tcp_current_list.publish(current_TCP_list)
        pub_tcp_current_pose.publish(tcp_current_pose)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    




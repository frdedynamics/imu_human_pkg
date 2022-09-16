#! /usr/bin/env python3

"""
This is a node to publish TCP force to create a COLIFT interrupt
"""

import rospy
from std_msgs.msg import Int64, String
import time

import rtde_receive


if __name__ == '__main__':
    rtde_r = rtde_receive.RTDEReceiveInterface("172.31.1.144")
    tcp_force = rtde_r.getActualTCPForce()
    print(tcp_force)
    




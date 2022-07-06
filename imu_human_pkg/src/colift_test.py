#! /usr/bin/env python3

"""
This is a node to test COLIFT state
"""

import rospy
from std_msgs.msg import Int64
import subprocess, time
from multiprocessing import Process, Queue

from rtde_control import RTDEControlInterface as RTDEControl
import rtde_receive

rtde_c = RTDEControl("172.31.1.144", RTDEControl.FLAG_USE_EXT_UR_CAP)
rtde_r = rtde_receive.RTDEReceiveInterface("172.31.1.144")

# colift_proc = subprocess.Popen(["sh", "../sh/myo.sh"])

test_count = Int64()


def force_apply():
    f = 15
    vector = [0, 0, 0, 0, 0, 0]
    type = 2 
    selection_vector = [1, 0, 0, 0, 0, 0]
    wrench = [-f, 0.0, 0.0, 0.0, 0.0, 0.0]
    limits = [0.5, 0.3, 0.3, 0.17, 0.17, 0.17]


    _curr_force = rtde_r.getActualTCPForce()
    # print(_curr_force)
    rtde_c.forceMode(vector, selection_vector, wrench, type, limits)

    time.sleep(2)

    rtde_c.forceModeStop()
    time.sleep(2)
    selection_vector = [1, 0, 0, 0, 0, 0]
    wrench = [f, 0.0, 0.0, 0.0, 0.0, 0.0]
    limits = [0.5, 0.3, 0.3, 0.17, 0.17, 0.17]
    rtde_c.forceMode(vector, selection_vector, wrench, type, limits)
    time.sleep(2)



def update(pub_test_cmd):
    force_apply()
    test_count.data = test_count.data + 1
    pub_test_cmd.publish(test_count)
    print(test_count)


def main(): 
    rospy.init_node('colift_test')
    print("Colift test node started")
    pub_test_cmd = rospy.Publisher('/test_in', Int64, queue_size=1)
    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            update(pub_test_cmd)
            rate.sleep()
    except KeyboardInterrupt:
        rtde_c.forceModeStop()
        rtde_c.disconnect()
        rospy.signal_shutdown("KeyboardInterrupt")
        # rtde_c.servoStop()
        raise
        

if __name__ == '__main__': main()
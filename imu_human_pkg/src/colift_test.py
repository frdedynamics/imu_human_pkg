#! /usr/bin/env python3

"""
This is a node to test COLIFT state
"""

import rospy
from std_msgs.msg import Int64, String
import time
from Classes.colift_thread_class import ForceThread

from rtde_control import RTDEControlInterface as RTDEControl
import rtde_receive

rtde_c = RTDEControl("172.31.1.144", RTDEControl.FLAG_USE_EXT_UR_CAP)
rtde_r = rtde_receive.RTDEReceiveInterface("172.31.1.144")

test_count = Int64()
dir_str = String()
prev_dir_str = String()
dir_str.data = "d"
prev_dir_str.data = "s"
dir_change_flag = False


def cb_dir_int(msg):
    global dir_str, prev_dir_str, dir_change_flag
    dir_str = msg
    if prev_dir_str.data == dir_str.data:
        # dir_change_flag = False
        pass
    else:
        dir_change_flag = True
        prev_dir_str = dir_str
        print("dir changed")


def update(pub_test_cmd):
    # force_apply()
    test_count.data = test_count.data + 1
    pub_test_cmd.publish(test_count)
    print(test_count)


def main():
    global dir_change_flag
    rospy.init_node('colift_test')
    print("Colift test node started")
    pub_test_cmd = rospy.Publisher('/test_in', Int64, queue_size=1)
    sub_dir_cmd = rospy.Subscriber('/dir_str', String, cb_dir_int)
    rate = rospy.Rate(100)

    force_thread = ForceThread(rtde_r=rtde_r, rtde_c=rtde_c, mode=dir_str.data)
    force_thread.join() # this blocks until the process terminates

    try:
        while not rospy.is_shutdown():
            if dir_change_flag:
                print("flag true")
                rtde_c.forceModeStop()
                if force_thread.is_alive():
                    Exception("ForceThread still alive")
                force_thread = ForceThread(rtde_r=rtde_r, rtde_c=rtde_c, mode=dir_str.data)
                force_thread.join()
                dir_change_flag = False
                prev = time.time()
            rate.sleep()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        rtde_c.forceModeStop()
        # rtde_c.servoStop()
        raise

    rtde_c.forceModeStop()
    print("Done")


if __name__ == '__main__':
    main()




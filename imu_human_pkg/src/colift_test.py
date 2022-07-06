#! /usr/bin/env python3

"""
This is a node to test COLIFT state
"""

import rospy
from std_msgs.msg import Int64, String
import subprocess, time

from rtde_control import RTDEControlInterface as RTDEControl
import rtde_receive

rtde_c = RTDEControl("172.31.1.144", RTDEControl.FLAG_USE_EXT_UR_CAP)
rtde_r = rtde_receive.RTDEReceiveInterface("172.31.1.144")

test_count = Int64()
dir_str = String()
prev_dir_str = String()
dir_str.data = 's'
prev_dir_str.data = 's'
dir_change_flag = True


def cb_dir_int(msg):
    global dir_str, prev_dir_str
    dir_str = msg
    if prev_dir_str.data == dir_str.data:
        dir_change_flag = False
    else:
        dir_change_flag = True
        prev_dir_str = dir_str



def update(pub_test_cmd):
    # force_apply()
    test_count.data = test_count.data + 1
    pub_test_cmd.publish(test_count)
    print(test_count)


if __name__ == '__main__':
    rospy.init_node('colift_test')
    print("Colift test node started")
    pub_test_cmd = rospy.Publisher('/test_in', Int64, queue_size=1)
    sub_dir_cmd = rospy.Subscriber('/dir_str', String, cb_dir_int)
    rate = rospy.Rate(10)

    # force_proc = subprocess.Popen(["./colift_test_class.py"], stdout=subprocess.PIPE) 
    force_proc = subprocess.Popen(["python3", "/Subprocess/colift_test_subprocess.py"], stdout=subprocess.PIPE) 

    try:
        while not rospy.is_shutdown():
            if dir_change_flag:
                try:
                    force_proc.kill()
                except AttributeError as e:
                    print("no force process found")
                prev = time.time()
                force_proc = subprocess.Popen(["python3", "Subprocess/colift_test_subprocess.py"], stdout=subprocess.PIPE) 
                # force_proc = subprocess.Popen(["./colift_test_class.py"], stdout=subprocess.PIPE) 
                # force_proc = subprocess.Popen(["./colift_test_class.py"], stdout=subprocess.PIPE) 
                # force_proc = subprocess.Popen(["python3", "Classes/colift_test_class.py", "15", dir_str.data, rtde_c], stdout=subprocess.PIPE) 
                out, err = force_proc.communicate()
                print(out,err)
                print("process", time.time()-prev)
            rate.sleep()
            print("here")
    except KeyboardInterrupt:
        
        rospy.signal_shutdown("KeyboardInterrupt")
        # rtde_c.servoStop()
        raise

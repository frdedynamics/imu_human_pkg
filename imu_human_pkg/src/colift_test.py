#! /usr/bin/env python3

"""
This is a node to test COLIFT state
"""

import rospy
from std_msgs.msg import Int64, String
import subprocess, time
from multiprocessing import Process, Queue

from rtde_control import RTDEControlInterface as RTDEControl
import rtde_receive

rtde_c = RTDEControl("172.31.1.144", RTDEControl.FLAG_USE_EXT_UR_CAP)
rtde_r = rtde_receive.RTDEReceiveInterface("172.31.1.144")

test_count = Int64()
dir_str = String()
prev_dir_str = String()
dir_str.data = 's'
prev_dir_str.data = 's'
dir_change_flag = False


def cb_dir_int(msg):
    global dir_str, prev_dir_str, dir_change_flag
    dir_str = msg
    if prev_dir_str.data == dir_str.data:
        dir_change_flag = False
    else:
        dir_change_flag = True
        prev_dir_str = dir_str
        print("dir changed")


def update(pub_test_cmd):
    # force_apply()
    test_count.data = test_count.data + 1
    pub_test_cmd.publish(test_count)
    print(test_count)


def my_function2(q, f, mode):
    vector = [0, 0, 0, 0, 0, 0]
    type = 2 
    selection_vector = [1, 0, 0, 0, 0, 0]
    # wrench = [-f, 0.0, 0.0, 0.0, 0.0, 0.0]
    limits = [0.5, 0.3, 0.3, 0.17, 0.17, 0.17]

    if mode =='l':
        wrench = [f, 0.0, 0.0, 0.0, 0.0, 0.0]
        # rtde_c.forceMode(vector, selection_vector, wrench, type, limits)
    elif mode =='r':
        wrench = [-f, 0.0, 0.0, 0.0, 0.0, 0.0]
        # rtde_c.forceMode(vector, selection_vector, wrench, type, limits)
    else:
        wrench = [-f, 0.0, 0.0, 0.0, 0.0, 0.0]
        # rtde_c.forceModeStop()

    _curr_force = rtde_r.getActualTCPForce()
    q.put(_curr_force)
    # return _curr_force


if __name__ == '__main__':
    rospy.init_node('colift_test')
    print("Colift test node started")
    pub_test_cmd = rospy.Publisher('/test_in', Int64, queue_size=1)
    sub_dir_cmd = rospy.Subscriber('/dir_str', String, cb_dir_int)
    rate = rospy.Rate(10)

    # Start process once
    queue = Queue()
    prev = time.time()
    force_proc = Process(target=my_function2, args=(queue, 15, 'l')) # force = 15
    print("process", time.time()-prev)
    prev = time.time()
    force_proc.daemon = True
    force_proc.start()
    print("start", time.time()-prev)
    prev = time.time()
    force_proc.join() # this blocks until the process terminates
    print("join", time.time()-prev)
    prev = time.time()
    result = queue.get()
    print("get", time.time()-prev)
    prev = time.time()


    # time.sleep(2)
    # rtde_c.forceModeStop()

    try:
        while not rospy.is_shutdown():
            print("FLAG:", dir_change_flag)
            if dir_change_flag:
                print("flag true")
                try:
                    # dir_change_flag = False
                    print("killing")
                    print(Queue.empty())
                    force_proc.kill()
                except AttributeError as e:
                    print("no force process found")
                prev = time.time()
                queue = Queue()
                force_proc = Process(target=my_function2, args=(queue, 15, 'l')) # force = 15
                print("process", time.time()-prev)
                prev = time.time()
                force_proc.daemon = True
                force_proc.start()
                print("start", time.time()-prev)
                prev = time.time()
                force_proc.join() # this blocks until the process terminates
                print("join", time.time()-prev)
                prev = time.time()
                result = queue.get()
                print("get", time.time()-prev)
                prev = time.time()
            rate.sleep()
            print("here")
    except KeyboardInterrupt:
        print(Queue.empty())
        force_proc.kill()
        rospy.signal_shutdown("KeyboardInterrupt")
        # rtde_c.servoStop()
        raise

    print(result)




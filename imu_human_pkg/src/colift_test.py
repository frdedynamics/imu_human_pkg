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

# must be a global function    
def my_function(q, f):
    # q.put(x + 100)
    # f=15
    vector = [0, 0, 0, 0, 0, 0]
    type = 2 
    selection_vector = [1, 0, 0, 0, 0, 0]
    wrench = [-f, 0.0, 0.0, 0.0, 0.0, 0.0]
    limits = [0.5, 0.3, 0.3, 0.17, 0.17, 0.17]

    _curr_force = rtde_r.getActualTCPForce()
    # rtde_c.forceMode(vector, selection_vector, wrench, type, limits)

    time.sleep(1)

    # rtde_c.forceModeStop()
    selection_vector = [1, 0, 0, 0, 0, 0]
    wrench = [f, 0.0, 0.0, 0.0, 0.0, 0.0]
    limits = [0.5, 0.3, 0.3, 0.17, 0.17, 0.17]
    # rtde_c.forceMode(vector, selection_vector, wrench, type, limits)
    time.sleep(1)
    # print(_curr_force)
    q.put(_curr_force)
    # return _curr_force


def my_function(q, f, mode):
    vector = [0, 0, 0, 0, 0, 0]
    type = 2 
    selection_vector = [1, 0, 0, 0, 0, 0]
    # wrench = [-f, 0.0, 0.0, 0.0, 0.0, 0.0]
    limits = [0.5, 0.3, 0.3, 0.17, 0.17, 0.17]

    if mode =='l':
        wrench = [f, 0.0, 0.0, 0.0, 0.0, 0.0]
        rtde_c.forceMode(vector, selection_vector, wrench, type, limits)
    elif mode =='r':
        wrench = [-f, 0.0, 0.0, 0.0, 0.0, 0.0]
        rtde_c.forceMode(vector, selection_vector, wrench, type, limits)
    else:
        rtde_c.forceModeStop()

    _curr_force = rtde_r.getActualTCPForce()
    q.put(_curr_force)
    # return _curr_force


if __name__ == '__main__':
    queue = Queue()
    prev = time.time()
    p = Process(target=my_function, args=(queue, 15, 'r')) # force = 15
    print("process", time.time()-prev)
    prev = time.time()
    p.daemon = True
    p.start()
    print("start", time.time()-prev)
    prev = time.time()
    p.join() # this blocks until the process terminates
    print("join", time.time()-prev)
    prev = time.time()
    result = queue.get()
    print("get", time.time()-prev)
    prev = time.time()
    time.sleep(2)
    rtde_c.forceModeStop()
    print(result)


'''
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


def force_apply(mode='run'):
    f=15
    
    while test_count.data < 4:
        vector = [0, 0, 0, 0, 0, 0]
        type = 2 
        selection_vector = [1, 0, 0, 0, 0, 0]
        wrench = [-f, 0.0, 0.0, 0.0, 0.0, 0.0]
        limits = [0.5, 0.3, 0.3, 0.17, 0.17, 0.17]

        _curr_force = rtde_r.getActualTCPForce()
        # rtde_c.forceMode(vector, selection_vector, wrench, type, limits)

        time.sleep(1)

        # rtde_c.forceModeStop()
        selection_vector = [1, 0, 0, 0, 0, 0]
        wrench = [f, 0.0, 0.0, 0.0, 0.0, 0.0]
        limits = [0.5, 0.3, 0.3, 0.17, 0.17, 0.17]
        # rtde_c.forceMode(vector, selection_vector, wrench, type, limits)
        time.sleep(1)
        print(_curr_force)
    # return _curr_force



def update(pub_test_cmd):
    # force_apply()
    test_count.data = test_count.data + 1
    pub_test_cmd.publish(test_count)
    print(test_count)


def main(): 
    rospy.init_node('colift_test')
    print("Colift test node started")
    pub_test_cmd = rospy.Publisher('/test_in', Int64, queue_size=1)
    rate = rospy.Rate(10)

    p = Process(target=force_apply(mode='run'))
    p.daemon = True
    p.start()
    p.join() # this blocks until the process terminates

    try:
        while not rospy.is_shutdown():
            # force_apply()
            update(pub_test_cmd)
            rate.sleep()
    except KeyboardInterrupt:
        # rtde_c.forceModeStop()
        # rtde_c.disconnect()
        p.terminate()
        rospy.signal_shutdown("KeyboardInterrupt")
        # rtde_c.servoStop()
        raise
        

if __name__ == '__main__': main()

'''
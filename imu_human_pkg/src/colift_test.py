#! /usr/bin/env python3

"""
This is a node to test COLIFT state
"""

import rospy

from rtde_control import RTDEControlInterface as RTDEControl
import rtde_receive

rtde_c = RTDEControl("172.31.1.144", RTDEControl.FLAG_USE_EXT_UR_CAP)
rtde_r = rtde_receive.RTDEReceiveInterface("172.31.1.144")

def force_apply():
    f = 15
    ''' Make force thingy here '''
    # vector = self.rtde_r.getActualTCPPose() # A pose vector that defines the force frame relative to the base frame.
    vector = [0, 0, 0, 0, 0, 0]
    selection_vector = [1, 1, 1, 0, 0, 0] # A 6d vector of 0s and 1s. 1 means that the robot will be compliant in the corresponding axis of the task frame
    wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # The forces/torques the robot will apply to its environment. The robot adjusts its position along/about compliant axis in order to achieve the specified force/torque. Values have no effect for non-compliant axes
    type = 2 # An integer [1;3] specifying how the robot interprets the force frame. 1: The force frame is transformed in a way such that its y-axis is aligned with a vector pointing from the robot tcp towards the origin of the force frame. 2: The force frame is not transformed. 3: The force frame is transformed in a way such that its x-axis is the projection of the robot tcp velocity vector onto the x-y plane of the force frame.
    limits = [f, f, f, 0.17, 0.17, 0.17]# (Float) 6d vector. For compliant axes, these values are the maximum allowed tcp speed along/about the axis. For non-compliant axes, these values are the maximum allowed deviation along/about an axis between the actual tcp position and the one set by the program.

    _curr_force = rtde_r.getActualTCPForce()
    print(_curr_force)


def update():
    force_apply()
    # print("updated")


def main(): 
    rospy.init_node('colift_test')
    print("Colift test node started")
    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            update()
            rate.sleep()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise
        

if __name__ == '__main__': main()
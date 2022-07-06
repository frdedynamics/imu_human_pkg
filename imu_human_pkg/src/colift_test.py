#! /usr/bin/env python3

"""
This is a node to test COLIFT state
"""

import rospy



def update():
    print("updated")


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
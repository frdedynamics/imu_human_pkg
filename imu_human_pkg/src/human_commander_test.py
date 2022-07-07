#! /usr/bin/env python3

"""
This is a node to test COLIFT state
"""

import rospy
from std_msgs.msg import Int64, String
import time
from Classes.human_commander_class import HumanCommander


def main(): 
	Human = HumanCommander(start_node=True)
	Human.init_subscribers_and_publishers()
	try:
		while not rospy.is_shutdown():
			Human.update()
			Human.r.sleep()
	except KeyboardInterrupt:
		rospy.signal_shutdown("KeyboardInterrupt")
		raise


if __name__ == '__main__': main()
#!/usr/bin/env python3

"""
Node runner for /Classes/hands_calibrate.py
"""

import rospy
from Classes.hands_calibrate import HandCalibrate
from math import pi


def main(): 
	Calibrator = HandCalibrate()
	Calibrator.init_node(rate=100.0)
	try:
		while not rospy.is_shutdown():
			Calibrator.update()
			Calibrator.rate.sleep()
	except KeyboardInterrupt:
		rospy.signal_shutdown("KeyboardInterrupt")
		raise


if __name__ == '__main__': main()
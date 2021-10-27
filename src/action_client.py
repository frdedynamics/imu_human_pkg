#! /usr/bin/env python3

import rospy
import actionlib
from arm_motion_controller_py3.msg import handCalibrationAction, handCalibrationGoal

def feedback_cb(msg):
 print('Feedback received:', msg)

def call_server():

    client = actionlib.SimpleActionClient('hand_calibration_as', handCalibrationAction)

    client.wait_for_server()

    goal = handCalibrationGoal()
    goal.calib_request = True

    client.send_goal(goal, feedback_cb=feedback_cb)

    client.wait_for_result()

    result = client.get_result()

    return result

if __name__ == '__main__':

    try:
        rospy.init_node('action_client')
        result = call_server()
        print('The result is:', result)
    except rospy.ROSInterruptException:
        print('Something went wrong:')

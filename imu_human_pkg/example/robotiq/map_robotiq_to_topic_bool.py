#!/usr/bin/env python3
""" Subscribes a boolean /cmd_grip_bool command and sends OPEN/CLOSE signal to Robotiq """
import sys
import rospy
from std_msgs.msg import Bool
from robotiq_urcap_control.msg import Robotiq2FGripper_robot_output as outputMsg

cmd_grip_pub = rospy.Publisher('/Robotiq2FGripperRobotOutput', outputMsg, queue_size=1)

def callback(data):
    msg = outputMsg()
    # print(data)
    if data.data:
        msg.rPR = 255  ## This should be 0/255 I suppose but check tomorrow
    else:
        msg.rPR = 0
    msg.rSP = 255
    msg.rFR = 255
    cmd_grip_pub.publish(msg)

def listener():
    rospy.init_node('grab_to_gripper_pub', anonymous=True)
    rospy.Subscriber("/cmd_grip_bool", Bool, callback)
    rospy.spin()


if __name__ == '__main__':
    MYARGV = rospy.myargv(argv=sys.argv)
    if len(MYARGV) > 1:
        rospy.loginfo("args??")
    rospy.loginfo("make the magic happen")
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

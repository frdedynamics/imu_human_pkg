#!/usr/bin/env python3

'''
Separated emg_to_gripper node from gripper commands, this is for emg_sum publish only.
Having this seperated than human commander is because of the unstability of Myo Armband

The EMG data is processed poorly so they are not reliable. Therefore, it works only as ON/OFF commands
The delay looks like inevitable if we want to process EMG data propely: https://www.youtube.com/watch?v=EnY56VFmAYY
'''
import rospy
from std_msgs.msg import Int16, Bool
from ros_myo.msg import EmgArray

# emg_data = []  ## change a numpy array if sum is too slow
emg_sum = 0 

def cb_emg(msg):
    global emg_sum
    emg_sum = sum(msg.data)


if __name__ == '__main__':
    rospy.init_node('emg_sum_node', anonymous=False)
    sub_emg = rospy.Subscriber('/myo_raw/myo_emg', EmgArray, cb_emg)
    pub_emg_sum = rospy.Publisher('/emg_sum', Int16, queue_size=1)
    rate = rospy.Rate(2.0)

    while not rospy.is_shutdown():
        try:
            pub_emg_sum.publish(emg_sum)  
            rate.sleep()      
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise

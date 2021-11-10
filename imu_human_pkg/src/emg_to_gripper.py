#!/usr/bin/env python3

'''
This node controls Robotiq gripper with ON/OFF commands via Myo Armband EMG data.
The EMG data is processed poorly so they are not reliable. Therefore, it works only as ON/OFF commands
The delay looks like inevitable if we want to process EMG data propely: https://www.youtube.com/watch?v=EnY56VFmAYY
'''
import rospy
from std_msgs.msg import Int64, Bool
from ros_myo.msg import EmgArray

# emg_data = []  ## change a numpy array if sum is too slow
emg_sum = 0 
gripper_open = True

def cb_emg(msg):
    global emg_sum
    emg_sum = sum(msg.data)


if __name__ == '__main__':
    rospy.init_node('emg_to_gripper', anonymous=True)
    sub_emg = rospy.Subscriber('/myo_raw/myo_emg', EmgArray, cb_emg)
    pub_emg_sum = rospy.Publisher('/emg_sum', Int64, queue_size=1)
    pub_gripper = rospy.Publisher('/cmd_grip_bool', Bool, queue_size=1)
    rate = rospy.Rate(2.0)

    while not rospy.is_shutdown():
        try:
            pub_emg_sum.publish(emg_sum)  
            print(emg_sum)
            if not emg_sum <   3000:
                if gripper_open:
                    gripper_open =  False
                    pub_gripper.publish(gripper_open)
                    rospy.sleep(2)
                else:
                    gripper_open = True
                    pub_gripper.publish(gripper_open)
                    rospy.sleep(2)

            rate.sleep()      
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise

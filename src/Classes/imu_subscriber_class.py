#!/usr/bin/env python3

"""
Subscribes 5 IMUS (LS, LE, RS, RE, RW(chest)). Publishes joint angles 5x2
Updated: 6 IMUS (LW) added.

"""

import sys, os
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_matrix as q2m
from tf.transformations import euler_from_quaternion as q2e
from tf.transformations import euler_from_matrix as m2e

sys.path.append('/home/gizem/catkin_ws/src/my_human_pkg/src/Classes')
from . import Kinematics_with_Quaternions as kinematic

_CALIBRATION_TH = 20
_ROSTIME_START = 0
prev = 0
now = 0

class IMUsubscriber:
    def __init__(self, rate=100, start_node=False):
        """Initializes the IMU data recording node."""

        if start_node == True:
            rospy.init_node("imu_subscriber_2arms")
            self.r = rospy.Rate(rate)

        self.q_chest_init = Quaternion(0, 0, 0, 1.0)
        self.l_q_shoulder_init = Quaternion(0, 0, 0, 1.0)
        self.l_q_elbow_init = Quaternion(0, 0, 0, 1.0)
        self.l_q_wrist_init = Quaternion(0, 0, 0, 1.0)
        self.r_q_shoulder_init = Quaternion(0, 0, 0, 1.0)
        self.r_q_elbow_init = Quaternion(0, 0, 0, 1.0)

        self.q_chest = Quaternion(0.0, 0.0, 0.707, 0.707)
        self.l_q_shoulder = Quaternion(0, 0, 0, 1.0)
        self.l_q_elbow = Quaternion(0, 0, 0, 1.0)
        self.l_q_wrist = Quaternion(0, 0, 0, 1.0)
        self.r_q_shoulder = Quaternion(0, 0, 0, 1.0)
        self.r_q_elbow = Quaternion(0, 0, 0, 1.0)

        self.acc_chest = Vector3()
        self.acc_ls = Vector3()
        self.acc_le = Vector3()
        self.acc_lw = Vector3()
        self.acc_rs = Vector3()
        self.acc_re = Vector3()

        self.gyro_chest = Vector3()
        self.gyro_ls = Vector3()
        self.gyro_le = Vector3()
        self.gyro_lw = Vector3()
        self.gyro_rs = Vector3()
        self.gyro_re = Vector3()

        self.human_joint_imu = JointState()
        self.human_joint_imu.name = [ 'spine_0', 'spine_1', 'spine_2',
                                      'left_shoulder_0', 'left_shoulder_1', 'left_shoulder_2', 'left_elbow_0', 'left_elbow_1', 'left_elbow_2',
                                      'right_shoulder_0', 'right_shoulder_1', 'right_shoulder_2', 'right_elbow_0', 'right_elbow_1', 'right_elbow_2',
                                      'left_wrist_0', 'left_wrist_1', 'left_wrist_2']
        self.human_joint_imu.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.motion_wrist_ori = Vector3()
        self.calibration_flag = 0
        self.runflag = False
        print("Created")


    def init_subscribers_and_publishers(self):
        self.pub = rospy.Publisher('/joint_states_human', JointState, queue_size=1)
        self.pub_lw_ori = rospy.Publisher('/lw_ori', Vector3, queue_size=1)
        self.pub_human_ori = rospy.Publisher('/human_ori', Quaternion, queue_size=1)
        self.sub_imu_c = rospy.Subscriber('/sensor_r_wrist', Imu, self.cb_imu_chest)
        self.sub_imu_ls = rospy.Subscriber('/sensor_l_shoulder', Imu, self.cb_imu_ls)
        self.sub_imu_le = rospy.Subscriber('/sensor_l_elbow', Imu, self.cb_imu_le)
        self.sub_imu_lw = rospy.Subscriber('/sensor_l_wrist', Imu, self.cb_imu_lw)
        self.sub_imu_rs = rospy.Subscriber('/sensor_r_shoulder', Imu, self.cb_imu_rs)
        self.sub_imu_re = rospy.Subscriber('/sensor_r_elbow', Imu, self.cb_imu_re)
        self.log_start_time = rospy.get_time()
        self.runflag = True
        _ROSTIME_START = rospy.get_time()
        print("Initialized")

    def update(self):
        # print self.calibration_flag
        self.human_joint_imu.header.stamp = rospy.Time.now()
        self.calibration_flag = self.calibration_flag + 1
        self.pub.publish(self.human_joint_imu)
        self.pub_human_ori.publish(self.q_chest)
        self.pub_lw_ori.publish(self.motion_wrist_ori)


    def cb_imu_chest(self, msg):
        self.chest_measurement = msg
        while self.calibration_flag < _CALIBRATION_TH:
            self.q_chest_init = kinematic.q_invert(self.chest_measurement.orientation)
            # print "calibrating chest"
        self.q_chest = kinematic.q_multiply(self.q_chest_init, self.chest_measurement.orientation)
        self.acc_chest = self.chest_measurement.linear_acceleration
        self.gyro_chest = self.chest_measurement.angular_velocity
        self.chest_angles = q2e(kinematic.q_tf_convert(self.q_chest), axes='sxyz')
        self.human_joint_imu.position[0] = self.chest_angles[2]  # pitch
        self.human_joint_imu.position[1] = - self.chest_angles[1]  # yaw
        self.human_joint_imu.position[2] = self.chest_angles[0]  # roll


    def cb_imu_ls(self, msg):
        self.ls_measurement = msg
        while self.calibration_flag < _CALIBRATION_TH:
            self.l_q_shoulder_init = kinematic.q_invert(self.ls_measurement.orientation)
        self.l_q_shoulder = kinematic.q_multiply(self.l_q_shoulder_init, self.ls_measurement.orientation)
        self.ls_angles = q2e(kinematic.q_tf_convert(self.l_q_shoulder), axes='sxyz')
        self.acc_ls = self.ls_measurement.linear_acceleration
        self.gyro_ls = self.ls_measurement.angular_velocity
        # l_q_shoulder_sensorframe = kinematic.q_multiply(kinematic.q_invert(self.q_chest), self.l_q_shoulder)
        # self.ls_angles = q2e(kinematic.q_tf_convert(l_q_shoulder_sensorframe), axes='sxyz')
        # Update joint angles
        self.human_joint_imu.position[5] = - self.ls_angles[0]  # pitch
        self.human_joint_imu.position[3] = - self.ls_angles[2]  # yaw
        self.human_joint_imu.position[4] = self.ls_angles[1]  # roll

    
    def cb_imu_le(self, msg):
        self.le_measurement = msg
        while self.calibration_flag < _CALIBRATION_TH:
            self.l_q_elbow_init = kinematic.q_invert(self.le_measurement.orientation)
        self.l_q_elbow = kinematic.q_multiply(self.l_q_elbow_init, self.le_measurement.orientation)
        l_q_elbow_sensorframe = kinematic.q_multiply(kinematic.q_invert(self.l_q_shoulder), self.l_q_elbow)
        self.le_angles = q2e(kinematic.q_tf_convert(l_q_elbow_sensorframe), axes='sxyz')
        self.acc_le = self.le_measurement.linear_acceleration
        self.gyro_le = self.le_measurement.angular_velocity
        # Update joint angles
        self.human_joint_imu.position[6] = -self.le_angles[2]  
        self.human_joint_imu.position[8] = self.le_angles[0]  
        # self.human_joint_imu.position[5] = self.le_angles[2]  
    
    def cb_imu_lw(self, msg):
        self.lw_measurement = msg
        while self.calibration_flag < _CALIBRATION_TH:
            self.l_q_wrist_init = kinematic.q_invert(self.lw_measurement.orientation)
            # print "calibrating elbow"
        self.l_q_wrist = kinematic.q_multiply(self.l_q_wrist_init, self.lw_measurement.orientation)
        l_q_wrist_sensorframe = kinematic.q_multiply(kinematic.q_invert(self.l_q_elbow), self.l_q_wrist)
        self.lw_angles = q2e(kinematic.q_tf_convert(l_q_wrist_sensorframe), axes='sxyz')
        self.acc_lw = self.lw_measurement.linear_acceleration
        self.gyro_lw = self.lw_measurement.angular_velocity
        # Update joint angles
        self.human_joint_imu.position[15] = -self.lw_angles[1]  # pitch
        self.human_joint_imu.position[16] = self.lw_angles[2]  # yaw
        self.human_joint_imu.position[17] = self.lw_angles[0]  # roll
        self.motion_wrist_ori.x = self.lw_angles[0]
        self.motion_wrist_ori.y = self.lw_angles[1]
        self.motion_wrist_ori.z = self.lw_angles[2]

    
    def cb_imu_rs(self, msg):
        self.rs_measurement = msg
        while self.calibration_flag < _CALIBRATION_TH:
            self.r_q_shoulder_init = kinematic.q_invert(self.rs_measurement.orientation)
        self.r_q_shoulder = kinematic.q_multiply(self.r_q_shoulder_init, self.rs_measurement.orientation)
        self.rs_angles = q2e(kinematic.q_tf_convert(self.r_q_shoulder), axes='sxyz')
        self.acc_rs = self.rs_measurement.linear_acceleration
        self.gyro_rs = self.rs_measurement.angular_velocity
        # r_q_shoulder_sensorframe = kinematic.q_multiply(kinematic.q_invert(self.r_q_chest), self.r_q_shoulder)
        # self.rs_angles = q2e(kinematic.q_tf_convert(r_q_shoulder_sensorframe), axes='sxyz')
        # Update joint angles
        self.human_joint_imu.position[11] = - self.rs_angles[0]  # pitch
        self.human_joint_imu.position[9] = self.rs_angles[2]  # yaw
        self.human_joint_imu.position[10] = self.rs_angles[1]  # roll

    
    def cb_imu_re(self, msg):
        self.re_measurement = msg
        while self.calibration_flag < _CALIBRATION_TH:
            self.r_q_elbow_init = kinematic.q_invert(self.re_measurement.orientation)
        self.r_q_elbow = kinematic.q_multiply(self.r_q_elbow_init, self.re_measurement.orientation)
        r_q_elbow_sensorframe = kinematic.q_multiply(kinematic.q_invert(self.r_q_shoulder), self.r_q_elbow)
        self.re_angles = q2e(kinematic.q_tf_convert(r_q_elbow_sensorframe), axes='sxyz')
        self.acc_re = self.re_measurement.linear_acceleration
        self.gyro_re = self.re_measurement.angular_velocity
        # Update joint angles
        self.human_joint_imu.position[12] = self.re_angles[2]  # pitch
        self.human_joint_imu.position[14] = - self.re_angles[0]  # yaw
        # self.human_joint_imu.position[11] = self.re_angles[2]  # roll
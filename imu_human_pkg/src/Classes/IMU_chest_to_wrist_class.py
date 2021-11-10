#!/usr/bin/env python3

"""
This is a subscriber. Subscribes the IMU readings and save into data_logger_module.

"""

# TODO: Elbow ve Wrist IMU ayni time frame'de aldigina emin ol.

# imports
# import Data.data_logger_module as data_logger

import sys, time

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_matrix as q2m
from tf.transformations import euler_from_quaternion as q2e
from tf.transformations import euler_from_matrix as m2e
from . import Kinematics_with_Quaternions as kinematic
from my_human_pkg.msg import test_msg

_CALIBRATION_TH = 20
_ROSTIME_START = 0
prev = 0
now = 0


class IMUsubscriber:
    def __init__(self, rate=100):
        """Initializes the IMU data recording node.
        @param DataLogger: the data logger object"""
        rospy.init_node("imu_subscriber")
        self.r = rospy.Rate(rate)

        self.q_chest_init = Quaternion(0, 0, 0, 1.0)
        self.q_shoulder_l_init = Quaternion(0, 0, 0, 1.0)
        self.q_elbow_l_init = Quaternion(0, 0, 0, 1.0)
        self.q_shoulder_r_init = Quaternion(0, 0, 0, 1.0)
        self.q_elbow_r_init = Quaternion(0, 0, 0, 1.0)

        self.q_chest = Quaternion(0, 0, 0, 1.0)
        self.q_shoulder = Quaternion(0, 0, 0, 1.0)
        self.q_elbow = Quaternion(0, 0, 0, 1.0)
        self.q_shoulder_l = Quaternion(0, 0, 0, 1.0)
        self.q_elbow_l = Quaternion(0, 0, 0, 1.0)

        self.acc_chest = Vector3()
        self.acc_shoulder_l = Vector3()
        self.acc_elbow_l = Vector3()
        self.acc_shoulder_r = Vector3()
        self.acc_elbow_r = Vector3()

        self.gyro_chest = Vector3()
        self.gyro_shoulder_l = Vector3()
        self.gyro_elbow_l = Vector3()
        self.gyro_shoulder_r = Vector3()
        self.gyro_elbow_r = Vector3()


        self.human_joint_imu = JointState()
        self.human_joint_imu.name = ['left_shoulder_2', 'left_shoulder_0', 'left_shoulder_1', 'left_elbow_2', 'left_elbow_0','right_shoulder_2', 'right_shoulder_0', 'right_shoulder_1', 'right_elbow_2', 'right_elbow_0']
        self.human_joint_imu.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.calibration_flag = 0
        self.runflag = False
        print("Created")

    def init_subscribers_and_publishers(self):
        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
        self.sub_imu_c = rospy.Subscriber('/sensor_r_wrist', Imu, self.cb_imu_chest)
        self.sub_imu_sl = rospy.Subscriber('/sensor_l_shoulder', Imu, self.cb_imu_shoulder_l)
        self.sub_imu_el = rospy.Subscriber('/sensor_l_elbow', Imu, self.cb_imu_elbow_l)
        self.sub_imu_sr = rospy.Subscriber('/sensor_r_shoulder', Imu, self.cb_imu_shoulder_r)
        self.sub_imu_se = rospy.Subscriber('/sensor_r_elbow', Imu, self.cb_imu_elbow_r)
        # self.log_start_time = rospy.get_time()
        self.runflag = True
        _ROSTIME_START = time.time()
        print("Publishers and subscribers are initialized")

    def update(self):
        # print self.calibration_flag
        self.human_joint_imu.header.stamp = rospy.Time.now()
        self.calibration_flag = self.calibration_flag + 1
        self.pub.publish(self.human_joint_imu)


    def cb_imu_chest(self, msg):
        self.chest_measurement = msg
        while self.calibration_flag < _CALIBRATION_TH:
            self.q_chest_init = kinematic.q_invert(self.chest_measurement.orientation)
            # print "calibrating chest"
        self.q_chest = kinematic.q_multiply(self.q_chest_init, self.chest_measurement.orientation)
        self.acc_chest = self.chest_measurement.linear_acceleration
        self.gyro_chest = self.chest_measurement.angular_velocity
        self.chest_angles = q2e(kinematic.q_tf_convert(self.q_chest), axes='sxyz')

    def cb_imu_shoulder_l(self, msg):
        self.shoulder_l_measurement = msg
        while self.calibration_flag < _CALIBRATION_TH:
            self.q_shoulder_l_init = kinematic.q_invert(self.shoulder_l_measurement.orientation)
            # print "calibrating shoulder"
        self.q_shoulder_l = kinematic.q_multiply(self.q_shoulder_l_init, self.shoulder_l_measurement.orientation)
        # print "q_measured: {0} \n q_init:{1}".format(self.shoulder_measurement.orientation, self.q_shoulder_init)
        self.shoulder_l_angles = q2e(kinematic.q_tf_convert(self.q_shoulder_l), axes='sxyz')
        self.acc_shoulder_l = self.shoulder_l_measurement.linear_acceleration
        self.gyro_shoulder_l = self.shoulder_l_measurement.angular_velocity
        # q_shoulder_sensorframe = kinematic.q_multiply(kinematic.q_invert(self.q_chest), self.q_shoulder)
        # self.shoulder_angles = q2e(kinematic.q_tf_convert(q_shoulder_sensorframe), axes='sxyz')
        # Update joint angles
        self.human_joint_imu.position[0] = self.shoulder_l_angles[0]  # pitch
        self.human_joint_imu.position[1] = self.shoulder_l_angles[1]  # yaw
        self.human_joint_imu.position[2] = self.shoulder_l_angles[2]  # roll

    def cb_imu_elbow_l(self, msg):
        self.elbow_l_measurement = msg
        while self.calibration_flag < _CALIBRATION_TH:
            self.q_elbow_l_init = kinematic.q_invert(self.elbow_l_measurement.orientation)
            # print "calibrating elbow"
        self.q_elbow_l = kinematic.q_multiply(self.q_elbow_l_init, self.elbow_l_measurement.orientation)
        q_elbow_l_sensorframe = kinematic.q_multiply(kinematic.q_invert(self.q_shoulder_l), self.q_elbow_l)
        self.elbow_l_angles = q2e(kinematic.q_tf_convert(q_elbow_l_sensorframe), axes='sxyz')
        self.acc_elbow_l = self.elbow_l_measurement.linear_acceleration
        self.gyro_elbow_l = self.elbow_l_measurement.angular_velocity
        # Update joint angles
        self.human_joint_imu.position[3] = self.elbow_l_angles[0]  # pitch
        self.human_joint_imu.position[4] = self.elbow_l_angles[1]  # yaw
        self.human_joint_imu.position[5] = self.elbow_l_angles[2]  # roll


    def cb_imu_shoulder_r(self, msg):
        self.shoulder_r_measurement = msg
        while self.calibration_flag < _CALIBRATION_TH:
            self.q_shoulder_r_init = kinematic.q_invert(self.shoulder_r_measurement.orientation)
            # print "calibrating shoulder"
        self.q_shoulder_r = kinematic.q_multiply(self.q_shoulder_r_init, self.shoulder_r_measurement.orientation)
        # print "q_measured: {0} \n q_init:{1}".format(self.shoulder_measurement.orientation, self.q_shoulder_init)
        self.shoulder_r_angles = q2e(kinematic.q_tf_convert(self.q_shoulder_r), axes='sxyz')
        self.acc_shoulder_r = self.shoulder_r_measurement.linear_acceleration
        self.gyro_shoulder_r = self.shoulder_r_measurement.angular_velocity
        # q_shoulder_sensorframe = kinematic.q_multiply(kinematic.q_invert(self.q_chest), self.q_shoulder)
        # self.shoulder_angles = q2e(kinematic.q_tf_convert(q_shoulder_sensorframe), axes='sxyz')
        # Update joint angles
        self.human_joint_imu.position[6] = self.shoulder_r_angles[0]  # pitch
        self.human_joint_imu.position[7] = self.shoulder_r_angles[1]  # yaw
        self.human_joint_imu.position[8] = self.shoulder_r_angles[2]  # roll

    def cb_imu_elbow_r(self, msg):
        self.elbow_r_measurement = msg
        while self.calibration_flag < _CALIBRATION_TH:
            self.q_elbow_r_init = kinematic.q_invert(self.elbow_r_measurement.orientation)
            # print "calibrating elbow"
        self.q_elbow_r = kinematic.q_multiply(self.q_elbow_r_init, self.elbow_r_measurement.orientation)
        q_elbow_r_sensorframe = kinematic.q_multiply(kinematic.q_invert(self.q_shoulder_r), self.q_elbow_r)
        self.elbow_r_angles = q2e(kinematic.q_tf_convert(q_elbow_r_sensorframe), axes='sxyz')
        self.acc_elbow_r = self.elbow_r_measurement.linear_acceleration
        self.gyro_elbow_r = self.elbow_r_measurement.angular_velocity
        # Update joint angles
        self.human_joint_imu.position[9] = self.elbow_r_angles[0]  # pitch
        self.human_joint_imu.position[10] = self.elbow_r_angles[1]  # yaw
        self.human_joint_imu.position[11] = self.elbow_r_angles[2]  # roll
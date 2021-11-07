#!/usr/bin/env python3
from PyQt5 import QtWidgets
from PyQt5 import QtGui
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *

import sys, random
from sensor_msgs.msg import JointState

from Classes.gui import Ui_MainWindow
import Classes.xacro_creator as xacro_creator

import roslaunch, rospy
import os

class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        self.setWindowTitle("IMU based HRC")
        self.setWindowIcon(QIcon('../config/hvlLogo.png'))  # Only for icon on toolbar

        self.l_upper_trunk.setText("0.510")
        self.l_upper_arm.setText("0.500")
        self.l_forearm.setText("0.299")
        self.l_hand.setText("0.203")

        self.radioButton_simulation.setChecked(True)
        self.comboBox_robot_name.addItems(['Panda', 'UR5e', 'KUKA iiwa'])
        self.lineEdit_robot_ip.setDisabled(True)  

        self.pushButton_calibrate_human.setToolTip("Set the human in N-pose\nCalibration will be completed\nin 3 seconds.")
        self.pushButton_connect_robot.setDisabled(True)
        self.pushButton_connect_robot.setToolTip("Simulation: Necessary models are uploaded to parameter server.\nReal robot: Real-time data exchange is set up.")
        self.pushButton_spawn_models.setDisabled(True)
        self.pushButton_spawn_models.setToolTip("Simulation-only: Human and robot models are spawn in Gazebo")
        self.pushButton_start_controllers.setDisabled(True)
        self.pushButton_spawn_models.setToolTip("Simulation-only: Human and robot controllers are started")
        self.pushButton_teleoperate.setDisabled(True)
        self.pushButton_teleoperate.setToolTip("Left hand orientation is mapped robot wrist joints\nRight hand respective position is mapped end-effector position")
        self.pushButton_colift.setDisabled(True)
        self.pushButton_colift.setToolTip("HRC process starts: IDLE-APPROACH-COLIFT-RELEASE")

        self.radioButton_real_robot.clicked.connect(self.real_robot_selected)
        self.radioButton_simulation.clicked.connect(self.simulation_selected)
        self.pushButton_calibrate_human.clicked.connect(self.calibrate_human_clicked)
        self.pushButton_connect_robot.clicked.connect(self.connect_robot_clicked)
        self.pushButton_spawn_models.clicked.connect(self.spawn_models_clicked)
        self.pushButton_start_controllers.clicked.connect(self.start_controllers_clicked)
        self.pushButton_teleoperate.clicked.connect(self.teleoperate_clicked)
        self.pushButton_colift.clicked.connect(self.colift_clicked)
    
    def real_robot_selected(self):
        self.textEdit.setText("Real robot selected")
        self.comboBox_robot_name.setDisabled(True)
        self.lineEdit_robot_ip.setEnabled(True)

    def simulation_selected(self):
        self.textEdit.setText("Simulation Selected")
        self.lineEdit_robot_ip.setDisabled(True)
        self.comboBox_robot_name.setEnabled(True)

    def calibrate_human_clicked(self):
        # Get package path
        command = 'rospack find imu_human_pkg'
        p = os.popen(command)
        self.pkg_path = str(p.read().split())[2:-2]

        self.pushButton_connect_robot.setEnabled(True)
        self.groupBox_human.setDisabled(True)
        self.groupBox_robots.setDisabled(True)
        self.textEdit.moveCursor(QTextCursor.End)
        self.textEdit.insertPlainText("Calibrating")
        self.textEdit.moveCursor(QTextCursor.End)

        xacro_creator.create_yaml(self.pkg_path, self.l_upper_trunk.text(), self.l_upper_arm.text(), self.l_forearm.text(), self.l_hand.text())

        rospy.init_node('imu_human_gui', anonymous=False)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [self.pkg_path+"/launch/human.launch"])
        self.launch.start()
        # self.sub_joint_states = rospy.Subscriber("/joint_states", JointState, self.cb_joint_states)
        rospy.loginfo("started")

    def connect_robot_clicked(self):
        # "TODO"
        self.pushButton_spawn_models.setEnabled(True)

    def spawn_models_clicked(self):
        # "TODO"
        self.pushButton_start_controllers.setEnabled(True)

    def start_controllers_clicked(self):
        # "TODO"
        self.pushButton_teleoperate.setEnabled(True)
        self.pushButton_colift.setEnabled(True)

    def teleoperate_clicked(self):
        pass

    def colift_clicked(self):
        pass

    def closeEvent(self, event):
        event.ignore()
        print("Killing all nodes. Please wait!")
        self.textEdit.insertPlainText("Killing all nodes. Please wait!")  # Why not working?
        try:
            self.launch.shutdown()
            p = os.popen("killall -9 gzserver gzclient")
            p = os.popen("killall -9 roscore")
        except AttributeError as e:
            print(e)
            print("Exiting...")
        event.accept()



stylesheet = """
    MainWindow {
        background-image: url("../config/hvlLogo.png");
        background-repeat: no-repeat; 
        padding: 25px 110px 4px 10px; /* top - right - bottom - left */
        background-repeat: no-repeat;
        background-position: right top;
        background-origin: content;
    }
"""

if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyleSheet(stylesheet)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())
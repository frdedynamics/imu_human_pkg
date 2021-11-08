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
from Classes.sensors import Ui_Dialog as SensorsDialog
from Classes.main_tool import Ui_Form as MainWidget

import roslaunch, rospy
import os


class MainTool(QWidget, MainWidget):
    def __init__(self, parent=None):
        super(MainTool, self).__init__(parent)
        self.setupUi(self)

class SensorTool(QDialog, SensorsDialog):
    def __init__(self, parent=None):
        super(SensorTool, self).__init__(parent)
        self.setupUi(self)

    # def closeEvent(self, a0: QtGui.QCloseEvent) -> None:
    #     return super().closeEvent(a0)


# class MainTool(QMainWindow, Ui_MainWindow):
#     def __init__(self, parent=None):
#         super(MainTool, self).__init__(parent)
#         self.setupUi(self)
#         self.setWindowIcon(QIcon('../config/hvlLogo.png'))  # Only for icon on toolbar
#         print("here")
        

class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.resize(800, 600)
        self.startMainTool()

    def startMainTool(self):
        self.MainTool = MainTool(self)
        self.setWindowTitle("IMU based HRC")
        self.setCentralWidget(self.MainTool)

        self.MainTool.l_upper_trunk.setText("0.510")
        self.MainTool.l_upper_arm.setText("0.500")
        self.MainTool.l_forearm.setText("0.299")
        self.MainTool.l_hand.setText("0.203")

        self.MainTool.radioButton_simulation.setChecked(True)
        self.MainTool.comboBox_robot_name.addItems(['Panda', 'UR5e', 'KUKA iiwa'])
        self.MainTool.lineEdit_robot_ip.setDisabled(True)  

        self.MainTool.pushButton_calibrate_human.setToolTip("Set the human in N-pose\nCalibration will be completed\nin 3 seconds.")
        self.MainTool.pushButton_connect_robot.setDisabled(True)
        self.MainTool.pushButton_connect_robot.setToolTip("Simulation: Necessary models are uploaded to parameter server.\nReal robot: Real-time data exchange is set up.")
        self.MainTool.pushButton_spawn_models.setDisabled(True)
        self.MainTool.pushButton_spawn_models.setToolTip("Simulation-only: Human and robot models are spawn in Gazebo")
        self.MainTool.pushButton_start_controllers.setDisabled(True)
        self.MainTool.pushButton_spawn_models.setToolTip("Simulation-only: Human and robot controllers are started")
        self.MainTool.pushButton_teleoperate.setDisabled(True)
        self.MainTool.pushButton_teleoperate.setToolTip("Left hand orientation is mapped robot wrist joints\nRight hand respective position is mapped end-effector position")
        self.MainTool.pushButton_colift.setDisabled(True)
        self.MainTool.pushButton_colift.setToolTip("HRC process starts: IDLE-APPROACH-COLIFT-RELEASE")


        self.MainTool.radioButton_real_robot.clicked.connect(self.real_robot_selected)
        self.MainTool.radioButton_simulation.clicked.connect(self.simulation_selected)
        self.MainTool.pushButton_calibrate_human.clicked.connect(self.calibrate_human_clicked)
        self.MainTool.pushButton_connect_robot.clicked.connect(self.connect_robot_clicked)
        self.MainTool.pushButton_spawn_models.clicked.connect(self.spawn_models_clicked)
        self.MainTool.pushButton_start_controllers.clicked.connect(self.start_controllers_clicked)
        self.MainTool.pushButton_teleoperate.clicked.connect(self.teleoperate_clicked)
        self.MainTool.pushButton_colift.clicked.connect(self.colift_clicked)
        # self.actionCustom_Sensor_Topics.triggered.connect(self.startSensorTool)
        self.show()
    
    def real_robot_selected(self):
        self.MainTool.textEdit.setText("Real robot selected")
        self.MainTool.comboBox_robot_name.setDisabled(True)
        self.MainTool.lineEdit_robot_ip.setEnabled(True)

    def simulation_selected(self):
        self.MainTool.textEdit.setText("Simulation Selected")
        self.MainTool.lineEdit_robot_ip.setDisabled(True)
        self.MainTool.comboBox_robot_name.setEnabled(True)

    def calibrate_human_clicked(self):
        # Get package path
        command = 'rospack find imu_human_pkg'
        p = os.popen(command)
        self.pkg_path = str(p.read().split())[2:-2]

        self.MainTool.pushButton_connect_robot.setEnabled(True)
        self.MainTool.groupBox_human.setDisabled(True)
        self.MainTool.groupBox_robots.setDisabled(True)
        self.MainTool.textEdit.moveCursor(QTextCursor.End)
        self.MainTool.textEdit.insertPlainText("Calibrating")
        self.MainTool.textEdit.moveCursor(QTextCursor.End)

        xacro_creator.create_human_yaml(self.pkg_path, self.MainTool.l_upper_trunk.text(), self.MainTool.l_upper_arm.text(), self.MainTool.l_forearm.text(), self.MainTool.l_hand.text())

        rospy.init_node('imu_human_gui', anonymous=False)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [self.pkg_path+"/launch/human.launch"])
        self.launch.start()
        # self.sub_joint_states = rospy.Subscriber("/joint_states", JointState, self.cb_joint_states)
        rospy.loginfo("Human calibrated")

    def connect_robot_clicked(self):
        # "TODO"
        self.MainTool.pushButton_spawn_models.setEnabled(True)

    def spawn_models_clicked(self):
        # "TODO"
        self.MainTool.pushButton_start_controllers.setEnabled(True)

    def start_controllers_clicked(self):
        # "TODO"
        self.MainTool.pushButton_teleoperate.setEnabled(True)
        self.MainTool.pushButton_colift.setEnabled(True)

    def teleoperate_clicked(self):
        pass

    def colift_clicked(self):
        pass


    def startSensorTool(self):
        self.SensorsTool = SensorTool(self)
        self.setWindowTitle("Sensors")
        self.setCentralWidget(self.SensorsTool)

        self.SensorsTool.lineEdit_chest.setText("sensor_r_wrist")
        self.SensorsTool.lineEdit_l_upper_arm.setText("sensor_l_shoulder")
        self.SensorsTool.lineEdit_l_forearm.setText("sensor_l_elbow")
        self.SensorsTool.lineEdit_l_hand.setText("sensor_l_wrist")
        self.SensorsTool.lineEdit_r_upper_arm.setText("sensor_r_shoulder")
        self.SensorsTool.lineEdit_r_forearm.setText("sensor_r_elbow")
        self.SensorsTool.lineEdit_emg.setText("???myo_emg")

        self.SensorsTool.buttonBox.accepted.connect(self.sensors_accepted)
        self.SensorsTool.buttonBox.rejected.connect(self.sensors_rejected)

        self.show()

    def sensors_accepted(self):
        print("OK clicked")
        # xacro_creator.create_sensor_list_yaml(self.lineEdit_chest.text(), self.lineEdit_l_upper_arm.text(), self.lineEdit_l_forearm.text(), self.lineEdit_l_hand.text(), self.lineEdit_r_upper_arm.text(), self.lineEdit_r_forearm.text(), self.lineEdit_emg.text())
        # TODO: Check if EMG is optional
        self.startMainTool()

    def sensors_rejected(self):
        print("Cancel clicked")
        

    def closeEvent(self, event):
        event.ignore()
        print("Killing all nodes. Please wait!")
        # self.MainTool.textEdit.insertPlainText("Killing all nodes. Please wait!")  # Why not working?
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
    # w.show()
    sys.exit(app.exec_())
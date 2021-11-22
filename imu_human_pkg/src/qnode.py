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

import roslaunch, rospy
import os, time


class SensorTool(QDialog, SensorsDialog):
    def __init__(self, parent=None):
        super(SensorTool, self).__init__(parent)
        self.setupUi(self)
        self.emg_flag = True
        


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
        self.simulated_flag = True
        self.emg_flag = False

        self.pushButton_calibrate_human.setToolTip("Set the human in N-pose\nCalibration will be completed\nin 3 seconds.")
        self.pushButton_connect_robot.setDisabled(True)
        self.pushButton_connect_robot.setToolTip("Simulation: Necessary models are uploaded to parameter server.\nHuman and robot is spawn in Gazebe.\nReal robot: Real-time data exchange is set up.")
        self.pushButton_robot_move_home.setDisabled(True)
        self.pushButton_robot_move_home.setToolTip("ONLY SIMULATION: move robot to home in Moveit")
        self.pushButton_map_poses.setDisabled(True)
        self.pushButton_map_poses.setToolTip("Human and robot initial poses are mapped")
        self.pushButton_teleoperate.setDisabled(True)
        self.pushButton_teleoperate.setToolTip("Left hand orientation is mapped robot wrist joints\nRight hand respective position is mapped end-effector position")
        self.pushButton_colift.setDisabled(True)
        self.pushButton_colift.setToolTip("HRC process starts: IDLE-APPROACH-COLIFT-RELEASE")

        self.radioButton_real_robot.clicked.connect(self.real_robot_selected)
        self.radioButton_simulation.clicked.connect(self.simulation_selected)
        self.pushButton_calibrate_human.clicked.connect(self.calibrate_human_clicked)
        self.pushButton_connect_robot.clicked.connect(self.connect_robot_clicked)
        self.pushButton_robot_move_home.clicked.connect(self.robot_move_home_clicked)
        self.pushButton_map_poses.clicked.connect(self.map_human_robot_poses_clicked)
        self.pushButton_teleoperate.clicked.connect(self.teleoperate_clicked)
        self.pushButton_colift.clicked.connect(self.colift_clicked)

        self.actionCustom_Sensor_Topics.triggered.connect(self.open_sensors_dialog)
        self.actionStart_Xsens.triggered.connect(self.start_xsens)

        # Start the main node
        command = 'rospack find imu_human_pkg'
        p = os.popen(command)
        self.pkg_path = str(p.read().split())[2:-2]
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        # rospy.init_node('imu_human_gui', anonymous=False)
    
    def real_robot_selected(self):
        self.textEdit_status.setText("Real robot selected")
        self.comboBox_robot_name.setDisabled(True)
        self.lineEdit_robot_ip.setEnabled(True)
        self.simulated_flag = False

    def simulation_selected(self):
        self.textEdit_status.setText("Simulation Selected")
        self.lineEdit_robot_ip.setDisabled(True)
        self.comboBox_robot_name.setEnabled(True)
        self.simulated_flag = True

    def calibrate_human_clicked(self):
        self.pushButton_connect_robot.setEnabled(True)
        self.groupBox_human.setDisabled(True)
        self.groupBox_robots.setDisabled(True)
        self.textEdit_status.moveCursor(QTextCursor.End)
        self.textEdit_status.insertPlainText("Calibrating")
        self.textEdit_status.moveCursor(QTextCursor.End)

        xacro_creator.create_human_yaml(self.pkg_path, self.l_upper_trunk.text(), self.l_upper_arm.text(), self.l_forearm.text(), self.l_hand.text())
        try:
            self.emg_flag = self.SensorsTool.emg_flag
        except AttributeError:
            print("Sensors not set. Taking default")
        
        if self.emg_flag:
            self.launch_human = roslaunch.parent.ROSLaunchParent(self.uuid, [self.pkg_path+"/launch/human_with_myo.launch"])
            self.launch_human.start()
            # Start another node
            node_emg_to_gripper = roslaunch.core.Node("imu_human_pkg", "emg_to_gripper.py",)
            self.launch_human.launch(node_emg_to_gripper)
            node_map_robotiq_to_topic_bool = roslaunch.core.Node("imu_human_pkg", "map_robotiq_to_topic_bool.py",)
            self.launch_human.launch(node_map_robotiq_to_topic_bool)
        else:
            self.launch_human = roslaunch.parent.ROSLaunchParent(self.uuid, [self.pkg_path+"/launch/human.launch"])
            self.launch_human.start()
        
        # self.sub_joint_states = rospy.Subscriber("/joint_states", JointState, self.cb_joint_states)
        rospy.loginfo("Human calibrated")

        #     def connect_robot_clicked(self):
        # # "TODO"
        # self.robot_name = self.comboBox_robot_name.currentText()
        # print(self.robot_name)
        # print(self.simulated_flag)
        # if self.simulated_flag == True:
        #     print("simulated robot selected")
        #     self.launch_panda = roslaunch.parent.ROSLaunchParent(self.uuid, [self.pkg_path+"/example/panda/launch/human_panda.launch"])
        #     self.launch_panda.start()
        # elif self.simulated_flag == False:
        #     print("real robot selected")
        # else:
        #     print("Something went wrong: Robot selection")

        # self.pushButton_spawn_models.setEnabled(True)


    def connect_robot_clicked(self):
        self.robot_name = self.comboBox_robot_name.currentText()
        print(self.simulated_flag)
        if self.simulated_flag == True:
            if self.robot_name == "Ur5e":
                command = 'rospack find ur_e_gazebo'
                p = os.popen(command)
                ur_gazebo_pkg_path = str(p.read().split())[2:-2]
                launch_ur5e_gazebo = roslaunch.parent.ROSLaunchParent(self.uuid, [ur_gazebo_pkg_path+"/launch/ur5e.launch"])
                launch_ur5e_gazebo.start()
            elif self.robot_name == "Panda":
                self.launch_panda = roslaunch.parent.ROSLaunchParent(self.uuid, [self.pkg_path+"/example/panda/launch/human_panda.launch"])
                self.launch_panda.start()
            
        elif self.simulated_flag == False:
            print("real robot selected")
            if self.robot_name == "Ur5e":
                print("RUN URTDE here")
                self.launch_real_ur = roslaunch.parent.ROSLaunchParent(self.uuid, [self.pkg_path+"/launch/ur5e_real.launch"])
                self.launch_real_ur.start()
                # TODO: make sure that initial pose movement is implemented here.

            elif self.robot_name == "Panda":
                print("Real Panda has not implemented yet. Starting simulation")
                # command = 'rospack find franka_gazebo'
                # p = os.popen(command)
                # ur_gazebo_pkg_path = str(p.read().split())[2:-2]

                # launch_panda_gazebo = roslaunch.parent.ROSLaunchParent(self.uuid, [ur_gazebo_pkg_path+"/launch/panda.launch"])
                # launch_panda_gazebo.start()
                self.simulated_flag = True
                self.launch_panda = roslaunch.parent.ROSLaunchParent(self.uuid, [self.pkg_path+"/example/panda/launch/human_panda.launch"])
                self.launch_panda.start()
        else:
            print("Something went wrong: Robot selection")

        if self.simulated_flag:
            self.pushButton_robot_move_home.setEnabled(True)
        else:
            self.pushButton_map_poses.setEnabled(True)

    def robot_move_home_clicked(self):
        # "TODO"
        print("Start Moveit launch here")
        print("Start move-node here")
        self.pushButton_map_poses.setEnabled(True)
        sys.exit()

    def map_human_robot_poses_clicked(self):
        # "TODO"
        self.pushButton_teleoperate.setEnabled(True)
        self.pushButton_colift.setEnabled(True)

    def teleoperate_clicked(self):
        pass

    def colift_clicked(self):
        pass

    def start_xsens(self):
        self.launch_xsens = roslaunch.parent.ROSLaunchParent(self.uuid, [self.pkg_path+"/launch/start_xsens.launch"])
        self.launch_xsens.start()
        # self.sub_joint_states = rospy.Subscriber("/joint_states", JointState, self.cb_joint_states)
        rospy.loginfo("Xsens node started")

    def open_sensors_dialog(self):
        self.Dialog = QtWidgets.QDialog()
        self.SensorsTool = SensorTool(self)
        self.SensorsTool.setupUi(self.Dialog)

        self.Dialog.setWindowTitle("Sensors")


        self.SensorsTool.label.setPixmap(QtGui.QPixmap(self.pkg_path+"/ui/fig/small_imu_placement50.png"))

        self.SensorsTool.lineEdit_chest.setText("sensor_r_wrist")
        self.SensorsTool.lineEdit_l_upper_arm.setText("sensor_l_shoulder")
        self.SensorsTool.lineEdit_l_forearm.setText("sensor_l_elbow")
        self.SensorsTool.lineEdit_l_hand.setText("sensor_l_wrist")
        self.SensorsTool.lineEdit_r_upper_arm.setText("sensor_r_shoulder")
        self.SensorsTool.lineEdit_r_forearm.setText("sensor_r_elbow")
        self.SensorsTool.lineEdit_emg.setText("myo_raw/myo_emg")
        self.SensorsTool.groupBox_emg.setDisabled(True)
        #TODO start a node with these

        self.SensorsTool.buttonBox.accepted.connect(self.sensors_accepted)
        self.SensorsTool.buttonBox.rejected.connect(self.sensors_rejected)
        self.SensorsTool.checkBox_emg.stateChanged.connect(self.state_changed)
    
        self.Dialog.show()
        self.Dialog.exec_()
    
    def state_changed(self, int):
        if self.SensorsTool.checkBox_emg.isChecked():
            print("EMG active!")
            self.SensorsTool.groupBox_emg.setEnabled(True)
            self.SensorsTool.emg_flag = True
        else:
            print("EMG deactive!")
            self.SensorsTool.groupBox_emg.setDisabled(True)
            self.SensorsTool.emg_flag = False
    
    def sensors_accepted(self):
        print("OK clicked")
        # Get package path (Double coded. Fix later #TODO)
        command = 'rospack find imu_human_pkg'
        p = os.popen(command)
        pkg_path = str(p.read().split())[2:-2]
        xacro_creator.create_sensors_yaml(pkg_path, self.SensorsTool.lineEdit_chest.text(), self.SensorsTool.lineEdit_l_upper_arm.text(), self.SensorsTool.lineEdit_l_forearm.text(), self.SensorsTool.lineEdit_l_hand.text(), self.SensorsTool.lineEdit_r_upper_arm.text(), self.SensorsTool.lineEdit_r_forearm.text(), self.SensorsTool.lineEdit_emg.text())
        self.Dialog.close()

    def sensors_rejected(self):
        print("Cancel clicked")
        self.Dialog.close()

    def closeEvent(self, event):
        event.ignore()
        print("Killing all nodes. Please wait!")
        self.textEdit_status.insertPlainText("Killing all nodes. Please wait!")  # Why not working?
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
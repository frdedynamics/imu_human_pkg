#!/usr/bin/env python3
from PyQt5 import QtWidgets
from PyQt5 import QtGui
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *

import sys, random
from sensor_msgs.msg import JointState

from Classes.gui import Ui_MainWindow

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

        self.radioButton_real_robot.clicked.connect(self.real_robot_selected)
        self.radioButton_simulation.clicked.connect(self.simulation_selected)
    
    def real_robot_selected(self):
        self.textEdit.setText("Real robot selected")
        self.comboBox_robot_name.setDisabled(True)
        self.lineEdit_robot_ip.setEnabled(True)

    def simulation_selected(self):
        self.textEdit.setText("Simulation Selected")
        self.lineEdit_robot_ip.setDisabled(True)
        self.comboBox_robot_name.setEnabled(True)



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
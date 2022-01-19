# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '/home/gizem/catkin_ws/src/imu_human_pkg/imu_human_pkg/ui/main.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.groupBox_human = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_human.setGeometry(QtCore.QRect(70, 20, 301, 151))
        self.groupBox_human.setObjectName("groupBox_human")
        self.label_2 = QtWidgets.QLabel(self.groupBox_human)
        self.label_2.setGeometry(QtCore.QRect(10, 30, 111, 17))
        self.label_2.setObjectName("label_2")
        self.label_3 = QtWidgets.QLabel(self.groupBox_human)
        self.label_3.setGeometry(QtCore.QRect(10, 60, 111, 17))
        self.label_3.setObjectName("label_3")
        self.label_4 = QtWidgets.QLabel(self.groupBox_human)
        self.label_4.setGeometry(QtCore.QRect(10, 90, 111, 17))
        self.label_4.setObjectName("label_4")
        self.label_5 = QtWidgets.QLabel(self.groupBox_human)
        self.label_5.setGeometry(QtCore.QRect(10, 120, 111, 17))
        self.label_5.setObjectName("label_5")
        self.l_upper_trunk = QtWidgets.QLineEdit(self.groupBox_human)
        self.l_upper_trunk.setGeometry(QtCore.QRect(160, 30, 113, 25))
        self.l_upper_trunk.setObjectName("l_upper_trunk")
        self.l_upper_arm = QtWidgets.QLineEdit(self.groupBox_human)
        self.l_upper_arm.setGeometry(QtCore.QRect(160, 60, 113, 25))
        self.l_upper_arm.setObjectName("l_upper_arm")
        self.l_forearm = QtWidgets.QLineEdit(self.groupBox_human)
        self.l_forearm.setGeometry(QtCore.QRect(160, 90, 113, 25))
        self.l_forearm.setObjectName("l_forearm")
        self.l_hand = QtWidgets.QLineEdit(self.groupBox_human)
        self.l_hand.setGeometry(QtCore.QRect(160, 120, 113, 25))
        self.l_hand.setObjectName("l_hand")
        self.groupBox_robots = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_robots.setGeometry(QtCore.QRect(70, 190, 641, 121))
        self.groupBox_robots.setObjectName("groupBox_robots")
        self.radioButton_simulation = QtWidgets.QRadioButton(self.groupBox_robots)
        self.radioButton_simulation.setGeometry(QtCore.QRect(170, 40, 112, 23))
        self.radioButton_simulation.setObjectName("radioButton_simulation")
        self.radioButton_real_robot = QtWidgets.QRadioButton(self.groupBox_robots)
        self.radioButton_real_robot.setGeometry(QtCore.QRect(470, 40, 112, 23))
        self.radioButton_real_robot.setObjectName("radioButton_real_robot")
        self.label = QtWidgets.QLabel(self.groupBox_robots)
        self.label.setGeometry(QtCore.QRect(360, 70, 67, 17))
        self.label.setObjectName("label")
        self.comboBox_robot_name = QtWidgets.QComboBox(self.groupBox_robots)
        self.comboBox_robot_name.setGeometry(QtCore.QRect(140, 70, 181, 25))
        self.comboBox_robot_name.setObjectName("comboBox_robot_name")
        self.label_6 = QtWidgets.QLabel(self.groupBox_robots)
        self.label_6.setGeometry(QtCore.QRect(30, 70, 91, 20))
        self.label_6.setObjectName("label_6")
        self.lineEdit_robot_ip = QtWidgets.QLineEdit(self.groupBox_robots)
        self.lineEdit_robot_ip.setGeometry(QtCore.QRect(440, 70, 181, 25))
        self.lineEdit_robot_ip.setObjectName("lineEdit_robot_ip")
        self.groupBox_actions = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_actions.setGeometry(QtCore.QRect(70, 320, 641, 221))
        self.groupBox_actions.setObjectName("groupBox_actions")
        self.pushButton_calibrate_human = QtWidgets.QPushButton(self.groupBox_actions)
        self.pushButton_calibrate_human.setGeometry(QtCore.QRect(10, 30, 181, 25))
        self.pushButton_calibrate_human.setObjectName("pushButton_calibrate_human")
        self.pushButton_human_robot_init = QtWidgets.QPushButton(self.groupBox_actions)
        self.pushButton_human_robot_init.setGeometry(QtCore.QRect(10, 70, 181, 25))
        self.pushButton_human_robot_init.setObjectName("pushButton_human_robot_init")
        self.textEdit_status = QtWidgets.QTextEdit(self.groupBox_actions)
        self.textEdit_status.setGeometry(QtCore.QRect(220, 60, 401, 111))
        self.textEdit_status.setObjectName("textEdit_status")
        self.pushButton_human_active = QtWidgets.QPushButton(self.groupBox_actions)
        self.pushButton_human_active.setGeometry(QtCore.QRect(10, 110, 181, 25))
        self.pushButton_human_active.setObjectName("pushButton_human_active")
        self.pushButton_robot_active = QtWidgets.QPushButton(self.groupBox_actions)
        self.pushButton_robot_active.setGeometry(QtCore.QRect(10, 150, 181, 25))
        self.pushButton_robot_active.setObjectName("pushButton_robot_active")
        self.label_debug = QtWidgets.QLabel(self.groupBox_actions)
        self.label_debug.setGeometry(QtCore.QRect(140, 180, 67, 31))
        self.label_debug.setObjectName("label_debug")
        self.pushButton_emg_reset = QtWidgets.QPushButton(self.groupBox_actions)
        self.pushButton_emg_reset.setGeometry(QtCore.QRect(220, 180, 89, 25))
        self.pushButton_emg_reset.setObjectName("pushButton_emg_reset")
        self.pushButton_stop = QtWidgets.QPushButton(self.groupBox_actions)
        self.pushButton_stop.setGeometry(QtCore.QRect(360, 180, 89, 25))
        self.pushButton_stop.setObjectName("pushButton_stop")
        self.pushButton_kill_all = QtWidgets.QPushButton(self.groupBox_actions)
        self.pushButton_kill_all.setGeometry(QtCore.QRect(510, 180, 89, 25))
        self.pushButton_kill_all.setObjectName("pushButton_kill_all")
        self.radioButton_teleop = QtWidgets.QRadioButton(self.groupBox_actions)
        self.radioButton_teleop.setGeometry(QtCore.QRect(250, 30, 112, 23))
        self.radioButton_teleop.setObjectName("radioButton_teleop")
        self.radioButton_colift = QtWidgets.QRadioButton(self.groupBox_actions)
        self.radioButton_colift.setGeometry(QtCore.QRect(440, 30, 112, 23))
        self.radioButton_colift.setObjectName("radioButton_colift")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 22))
        self.menubar.setObjectName("menubar")
        self.menuSensors = QtWidgets.QMenu(self.menubar)
        self.menuSensors.setObjectName("menuSensors")
        self.menuLog = QtWidgets.QMenu(self.menubar)
        self.menuLog.setObjectName("menuLog")
        self.menuHelp = QtWidgets.QMenu(self.menubar)
        self.menuHelp.setObjectName("menuHelp")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.actionStart_Xsens = QtWidgets.QAction(MainWindow)
        self.actionStart_Xsens.setObjectName("actionStart_Xsens")
        self.actionLog_preferences = QtWidgets.QAction(MainWindow)
        self.actionLog_preferences.setObjectName("actionLog_preferences")
        self.actionSensors_list = QtWidgets.QAction(MainWindow)
        self.actionSensors_list.setObjectName("actionSensors_list")
        self.actionRobots_list = QtWidgets.QAction(MainWindow)
        self.actionRobots_list.setObjectName("actionRobots_list")
        self.actionIMU_attachment = QtWidgets.QAction(MainWindow)
        self.actionIMU_attachment.setObjectName("actionIMU_attachment")
        self.actionCustom_Sensor_Topics = QtWidgets.QAction(MainWindow)
        self.actionCustom_Sensor_Topics.setObjectName("actionCustom_Sensor_Topics")
        self.menuSensors.addAction(self.actionStart_Xsens)
        self.menuSensors.addAction(self.actionCustom_Sensor_Topics)
        self.menuLog.addAction(self.actionLog_preferences)
        self.menuHelp.addAction(self.actionSensors_list)
        self.menuHelp.addAction(self.actionRobots_list)
        self.menuHelp.addAction(self.actionIMU_attachment)
        self.menubar.addAction(self.menuSensors.menuAction())
        self.menubar.addAction(self.menuLog.menuAction())
        self.menubar.addAction(self.menuHelp.menuAction())

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.groupBox_human.setTitle(_translate("MainWindow", "Human Measurements"))
        self.label_2.setText(_translate("MainWindow", "Upper trunk:"))
        self.label_3.setText(_translate("MainWindow", "Upper arm:"))
        self.label_4.setText(_translate("MainWindow", "Forearm:"))
        self.label_5.setText(_translate("MainWindow", "Hand:"))
        self.groupBox_robots.setTitle(_translate("MainWindow", "Robots"))
        self.radioButton_simulation.setText(_translate("MainWindow", "Simulation"))
        self.radioButton_real_robot.setText(_translate("MainWindow", "Real robot"))
        self.label.setText(_translate("MainWindow", "Robot IP:"))
        self.label_6.setText(_translate("MainWindow", "Robot Name:"))
        self.groupBox_actions.setTitle(_translate("MainWindow", "Actions"))
        self.pushButton_calibrate_human.setText(_translate("MainWindow", "Calibrate Human"))
        self.pushButton_human_robot_init.setText(_translate("MainWindow", "Human-Robot Initiate"))
        self.pushButton_human_active.setText(_translate("MainWindow", "Human Activate"))
        self.pushButton_robot_active.setText(_translate("MainWindow", "Robot Activate"))
        self.label_debug.setText(_translate("MainWindow", "Debug:"))
        self.pushButton_emg_reset.setText(_translate("MainWindow", "EMG Reset"))
        self.pushButton_stop.setText(_translate("MainWindow", "STOP"))
        self.pushButton_kill_all.setText(_translate("MainWindow", "Kill all"))
        self.radioButton_teleop.setText(_translate("MainWindow", "Teleoperate"))
        self.radioButton_colift.setText(_translate("MainWindow", "Colift"))
        self.menuSensors.setTitle(_translate("MainWindow", "Sensors"))
        self.menuLog.setTitle(_translate("MainWindow", "Log"))
        self.menuHelp.setTitle(_translate("MainWindow", "Help"))
        self.actionStart_Xsens.setText(_translate("MainWindow", "Start Xsens"))
        self.actionLog_preferences.setText(_translate("MainWindow", "Log preferences"))
        self.actionSensors_list.setText(_translate("MainWindow", "Sensors list"))
        self.actionRobots_list.setText(_translate("MainWindow", "Robots list"))
        self.actionIMU_attachment.setText(_translate("MainWindow", "IMU attachment"))
        self.actionCustom_Sensor_Topics.setText(_translate("MainWindow", "Custom Sensor Topics"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

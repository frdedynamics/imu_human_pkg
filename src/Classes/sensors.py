# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'sensors.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(432, 385)
        self.groupBox_imu = QtWidgets.QGroupBox(Dialog)
        self.groupBox_imu.setGeometry(QtCore.QRect(20, 0, 381, 221))
        self.groupBox_imu.setObjectName("groupBox_imu")
        self.label_2 = QtWidgets.QLabel(self.groupBox_imu)
        self.label_2.setGeometry(QtCore.QRect(10, 30, 111, 17))
        self.label_2.setObjectName("label_2")
        self.label_3 = QtWidgets.QLabel(self.groupBox_imu)
        self.label_3.setGeometry(QtCore.QRect(10, 60, 121, 17))
        self.label_3.setObjectName("label_3")
        self.lineEdit_chest = QtWidgets.QLineEdit(self.groupBox_imu)
        self.lineEdit_chest.setGeometry(QtCore.QRect(160, 30, 201, 25))
        self.lineEdit_chest.setObjectName("lineEdit_chest")
        self.lineEdit_l_upper_arm = QtWidgets.QLineEdit(self.groupBox_imu)
        self.lineEdit_l_upper_arm.setGeometry(QtCore.QRect(160, 60, 201, 25))
        self.lineEdit_l_upper_arm.setObjectName("lineEdit_l_upper_arm")
        self.lineEdit_l_forearm = QtWidgets.QLineEdit(self.groupBox_imu)
        self.lineEdit_l_forearm.setGeometry(QtCore.QRect(160, 90, 201, 25))
        self.lineEdit_l_forearm.setObjectName("lineEdit_l_forearm")
        self.lineEdit_l_hand = QtWidgets.QLineEdit(self.groupBox_imu)
        self.lineEdit_l_hand.setGeometry(QtCore.QRect(160, 120, 201, 25))
        self.lineEdit_l_hand.setText("")
        self.lineEdit_l_hand.setObjectName("lineEdit_l_hand")
        self.label_4 = QtWidgets.QLabel(self.groupBox_imu)
        self.label_4.setGeometry(QtCore.QRect(10, 90, 121, 17))
        self.label_4.setObjectName("label_4")
        self.label_5 = QtWidgets.QLabel(self.groupBox_imu)
        self.label_5.setGeometry(QtCore.QRect(10, 120, 121, 17))
        self.label_5.setObjectName("label_5")
        self.lineEdit_r_forearm = QtWidgets.QLineEdit(self.groupBox_imu)
        self.lineEdit_r_forearm.setGeometry(QtCore.QRect(160, 180, 201, 25))
        self.lineEdit_r_forearm.setObjectName("lineEdit_r_forearm")
        self.label_6 = QtWidgets.QLabel(self.groupBox_imu)
        self.label_6.setGeometry(QtCore.QRect(10, 150, 121, 17))
        self.label_6.setObjectName("label_6")
        self.label_7 = QtWidgets.QLabel(self.groupBox_imu)
        self.label_7.setGeometry(QtCore.QRect(10, 180, 121, 17))
        self.label_7.setObjectName("label_7")
        self.lineEdit_r_upper_arm = QtWidgets.QLineEdit(self.groupBox_imu)
        self.lineEdit_r_upper_arm.setGeometry(QtCore.QRect(160, 150, 201, 25))
        self.lineEdit_r_upper_arm.setObjectName("lineEdit_r_upper_arm")
        self.groupBox_emg = QtWidgets.QGroupBox(Dialog)
        self.groupBox_emg.setGeometry(QtCore.QRect(20, 230, 381, 61))
        self.groupBox_emg.setObjectName("groupBox_emg")
        self.lineEdit_emg = QtWidgets.QLineEdit(self.groupBox_emg)
        self.lineEdit_emg.setGeometry(QtCore.QRect(160, 30, 201, 25))
        self.lineEdit_emg.setObjectName("lineEdit_emg")
        self.label_8 = QtWidgets.QLabel(self.groupBox_emg)
        self.label_8.setGeometry(QtCore.QRect(10, 30, 111, 17))
        self.label_8.setObjectName("label_8")
        self.buttonBox = QtWidgets.QDialogButtonBox(Dialog)
        self.buttonBox.setGeometry(QtCore.QRect(130, 320, 166, 25))
        self.buttonBox.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel|QtWidgets.QDialogButtonBox.Ok)
        self.buttonBox.setObjectName("buttonBox")

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.groupBox_imu.setTitle(_translate("Dialog", "IMU Topics"))
        self.label_2.setText(_translate("Dialog", "Chest:"))
        self.label_3.setText(_translate("Dialog", "Left Upper Arm:"))
        self.label_4.setText(_translate("Dialog", "Left Forearm:"))
        self.label_5.setText(_translate("Dialog", "Left Hand:"))
        self.label_6.setText(_translate("Dialog", "Right Upper Arm:"))
        self.label_7.setText(_translate("Dialog", "Right Forearm:"))
        self.groupBox_emg.setTitle(_translate("Dialog", "EMG Topic (Optional)"))
        self.label_8.setText(_translate("Dialog", "EMG:"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Dialog = QtWidgets.QDialog()
    ui = Ui_Dialog()
    ui.setupUi(Dialog)
    Dialog.show()
    sys.exit(app.exec_())

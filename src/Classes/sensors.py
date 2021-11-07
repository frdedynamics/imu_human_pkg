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
        Dialog.resize(400, 300)
        self.groupBox_human = QtWidgets.QGroupBox(Dialog)
        self.groupBox_human.setGeometry(QtCore.QRect(20, 0, 301, 221))
        self.groupBox_human.setObjectName("groupBox_human")
        self.label_2 = QtWidgets.QLabel(self.groupBox_human)
        self.label_2.setGeometry(QtCore.QRect(10, 30, 111, 17))
        self.label_2.setObjectName("label_2")
        self.label_3 = QtWidgets.QLabel(self.groupBox_human)
        self.label_3.setGeometry(QtCore.QRect(10, 60, 121, 17))
        self.label_3.setObjectName("label_3")
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
        self.l_hand.setText("")
        self.l_hand.setObjectName("l_hand")
        self.label_4 = QtWidgets.QLabel(self.groupBox_human)
        self.label_4.setGeometry(QtCore.QRect(10, 90, 121, 17))
        self.label_4.setObjectName("label_4")
        self.label_5 = QtWidgets.QLabel(self.groupBox_human)
        self.label_5.setGeometry(QtCore.QRect(10, 120, 121, 17))
        self.label_5.setObjectName("label_5")
        self.l_forearm_2 = QtWidgets.QLineEdit(self.groupBox_human)
        self.l_forearm_2.setGeometry(QtCore.QRect(160, 180, 113, 25))
        self.l_forearm_2.setObjectName("l_forearm_2")
        self.label_6 = QtWidgets.QLabel(self.groupBox_human)
        self.label_6.setGeometry(QtCore.QRect(10, 150, 121, 17))
        self.label_6.setObjectName("label_6")
        self.label_7 = QtWidgets.QLabel(self.groupBox_human)
        self.label_7.setGeometry(QtCore.QRect(10, 180, 121, 17))
        self.label_7.setObjectName("label_7")
        self.l_upper_arm_2 = QtWidgets.QLineEdit(self.groupBox_human)
        self.l_upper_arm_2.setGeometry(QtCore.QRect(160, 150, 113, 25))
        self.l_upper_arm_2.setObjectName("l_upper_arm_2")
        self.groupBox_human_2 = QtWidgets.QGroupBox(Dialog)
        self.groupBox_human_2.setGeometry(QtCore.QRect(20, 230, 301, 61))
        self.groupBox_human_2.setObjectName("groupBox_human_2")
        self.l_upper_trunk_2 = QtWidgets.QLineEdit(self.groupBox_human_2)
        self.l_upper_trunk_2.setGeometry(QtCore.QRect(160, 30, 113, 25))
        self.l_upper_trunk_2.setObjectName("l_upper_trunk_2")
        self.l_forearm_4 = QtWidgets.QLineEdit(self.groupBox_human_2)
        self.l_forearm_4.setGeometry(QtCore.QRect(160, 180, 113, 25))
        self.l_forearm_4.setObjectName("l_forearm_4")
        self.label_13 = QtWidgets.QLabel(self.groupBox_human_2)
        self.label_13.setGeometry(QtCore.QRect(10, 180, 121, 17))
        self.label_13.setObjectName("label_13")
        self.label_8 = QtWidgets.QLabel(self.groupBox_human_2)
        self.label_8.setGeometry(QtCore.QRect(10, 30, 111, 17))
        self.label_8.setObjectName("label_8")

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.groupBox_human.setTitle(_translate("Dialog", "IMU Topics"))
        self.label_2.setText(_translate("Dialog", "Chest:"))
        self.label_3.setText(_translate("Dialog", "Left Upper Arm:"))
        self.label_4.setText(_translate("Dialog", "Left Forearm:"))
        self.label_5.setText(_translate("Dialog", "Left Hand:"))
        self.label_6.setText(_translate("Dialog", "Right Upper Arm:"))
        self.label_7.setText(_translate("Dialog", "Right Forearm:"))
        self.groupBox_human_2.setTitle(_translate("Dialog", "EMG Topic (Optional)"))
        self.label_13.setText(_translate("Dialog", "Right Forearm:"))
        self.label_8.setText(_translate("Dialog", "EMG:"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Dialog = QtWidgets.QDialog()
    ui = Ui_Dialog()
    ui.setupUi(Dialog)
    Dialog.show()
    sys.exit(app.exec_())

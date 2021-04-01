# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'gui.ui'
##
## Created by: Qt User Interface Compiler version 5.15.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *


class Ui_Form(object):
    def setupUi(self, Form):
        if not Form.objectName():
            Form.setObjectName(u"Form")
        Form.resize(963, 751)
        self.verticalLayout_3 = QVBoxLayout(Form)
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.horizontalLayout_3 = QHBoxLayout()
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.groupBox_6 = QGroupBox(Form)
        self.groupBox_6.setObjectName(u"groupBox_6")
        self.verticalLayout_4 = QVBoxLayout(self.groupBox_6)
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.plainTextEdit_received = QPlainTextEdit(self.groupBox_6)
        self.plainTextEdit_received.setObjectName(u"plainTextEdit_received")
        self.plainTextEdit_received.setMaximumSize(QSize(16777215, 16777215))
        self.plainTextEdit_received.setReadOnly(True)

        self.verticalLayout_4.addWidget(self.plainTextEdit_received)


        self.horizontalLayout_3.addWidget(self.groupBox_6)

        self.groupBox_8 = QGroupBox(Form)
        self.groupBox_8.setObjectName(u"groupBox_8")
        self.verticalLayout_5 = QVBoxLayout(self.groupBox_8)
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.plainTextEdit_send = QPlainTextEdit(self.groupBox_8)
        self.plainTextEdit_send.setObjectName(u"plainTextEdit_send")
        self.plainTextEdit_send.setMaximumSize(QSize(16777215, 16777215))
        self.plainTextEdit_send.setReadOnly(True)

        self.verticalLayout_5.addWidget(self.plainTextEdit_send)


        self.horizontalLayout_3.addWidget(self.groupBox_8)

        self.groupBox_7 = QGroupBox(Form)
        self.groupBox_7.setObjectName(u"groupBox_7")
        self.verticalLayout_6 = QVBoxLayout(self.groupBox_7)
        self.verticalLayout_6.setObjectName(u"verticalLayout_6")
        self.plainTextEdit_align = QPlainTextEdit(self.groupBox_7)
        self.plainTextEdit_align.setObjectName(u"plainTextEdit_align")
        self.plainTextEdit_align.setMaximumSize(QSize(16777215, 16777215))

        self.verticalLayout_6.addWidget(self.plainTextEdit_align)


        self.horizontalLayout_3.addWidget(self.groupBox_7)


        self.verticalLayout_3.addLayout(self.horizontalLayout_3)

        self.horizontalLayout_2 = QHBoxLayout()
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.groupBox_3 = QGroupBox(Form)
        self.groupBox_3.setObjectName(u"groupBox_3")
        sizePolicy = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.groupBox_3.sizePolicy().hasHeightForWidth())
        self.groupBox_3.setSizePolicy(sizePolicy)
        self.verticalLayout = QVBoxLayout(self.groupBox_3)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.label_image = QLabel(self.groupBox_3)
        self.label_image.setObjectName(u"label_image")
        self.label_image.setMinimumSize(QSize(640, 480))
        self.label_image.setMaximumSize(QSize(640, 480))

        self.verticalLayout.addWidget(self.label_image)

        self.horizontalLayout_4 = QHBoxLayout()
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.radioButton_test = QRadioButton(self.groupBox_3)
        self.radioButton_test.setObjectName(u"radioButton_test")
        self.radioButton_test.setChecked(False)

        self.horizontalLayout_4.addWidget(self.radioButton_test)

        self.radioButton_color = QRadioButton(self.groupBox_3)
        self.radioButton_color.setObjectName(u"radioButton_color")
        self.radioButton_color.setChecked(True)

        self.horizontalLayout_4.addWidget(self.radioButton_color)

        self.radioButton_depth = QRadioButton(self.groupBox_3)
        self.radioButton_depth.setObjectName(u"radioButton_depth")

        self.horizontalLayout_4.addWidget(self.radioButton_depth)

        self.horizontalSpacer_2 = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout_4.addItem(self.horizontalSpacer_2)


        self.verticalLayout.addLayout(self.horizontalLayout_4)


        self.horizontalLayout_2.addWidget(self.groupBox_3)

        self.verticalLayout_2 = QVBoxLayout()
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.groupBox = QGroupBox(Form)
        self.groupBox.setObjectName(u"groupBox")
        self.gridLayout = QGridLayout(self.groupBox)
        self.gridLayout.setObjectName(u"gridLayout")
        self.pushButton_rleft = QPushButton(self.groupBox)
        self.pushButton_rleft.setObjectName(u"pushButton_rleft")

        self.gridLayout.addWidget(self.pushButton_rleft, 0, 0, 1, 1)

        self.pushButton_forward = QPushButton(self.groupBox)
        self.pushButton_forward.setObjectName(u"pushButton_forward")

        self.gridLayout.addWidget(self.pushButton_forward, 0, 1, 1, 1)

        self.pushButton_rright = QPushButton(self.groupBox)
        self.pushButton_rright.setObjectName(u"pushButton_rright")

        self.gridLayout.addWidget(self.pushButton_rright, 0, 2, 1, 1)

        self.pushButton_left = QPushButton(self.groupBox)
        self.pushButton_left.setObjectName(u"pushButton_left")

        self.gridLayout.addWidget(self.pushButton_left, 1, 0, 1, 1)

        self.pushButton_back = QPushButton(self.groupBox)
        self.pushButton_back.setObjectName(u"pushButton_back")

        self.gridLayout.addWidget(self.pushButton_back, 1, 1, 1, 1)

        self.pushButton_right = QPushButton(self.groupBox)
        self.pushButton_right.setObjectName(u"pushButton_right")

        self.gridLayout.addWidget(self.pushButton_right, 1, 2, 1, 1)


        self.verticalLayout_2.addWidget(self.groupBox)

        self.groupBox_2 = QGroupBox(Form)
        self.groupBox_2.setObjectName(u"groupBox_2")
        self.gridLayout_2 = QGridLayout(self.groupBox_2)
        self.gridLayout_2.setObjectName(u"gridLayout_2")
        self.pushButton_body_lift = QPushButton(self.groupBox_2)
        self.pushButton_body_lift.setObjectName(u"pushButton_body_lift")

        self.gridLayout_2.addWidget(self.pushButton_body_lift, 0, 0, 1, 1)

        self.pushButton_camera_lift = QPushButton(self.groupBox_2)
        self.pushButton_camera_lift.setObjectName(u"pushButton_camera_lift")

        self.gridLayout_2.addWidget(self.pushButton_camera_lift, 1, 0, 1, 1)

        self.pushButton_grasp = QPushButton(self.groupBox_2)
        self.pushButton_grasp.setObjectName(u"pushButton_grasp")

        self.gridLayout_2.addWidget(self.pushButton_grasp, 0, 1, 1, 1)

        self.pushButton_push = QPushButton(self.groupBox_2)
        self.pushButton_push.setObjectName(u"pushButton_push")

        self.gridLayout_2.addWidget(self.pushButton_push, 0, 2, 1, 1)

        self.pushButton_auto_align = QPushButton(self.groupBox_2)
        self.pushButton_auto_align.setObjectName(u"pushButton_auto_align")

        self.gridLayout_2.addWidget(self.pushButton_auto_align, 1, 2, 1, 1)

        self.pushButton_flip = QPushButton(self.groupBox_2)
        self.pushButton_flip.setObjectName(u"pushButton_flip")

        self.gridLayout_2.addWidget(self.pushButton_flip, 1, 1, 1, 1)


        self.verticalLayout_2.addWidget(self.groupBox_2)

        self.groupBox_5 = QGroupBox(Form)
        self.groupBox_5.setObjectName(u"groupBox_5")
        self.horizontalLayout = QHBoxLayout(self.groupBox_5)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.slider_belt = QSlider(self.groupBox_5)
        self.slider_belt.setObjectName(u"slider_belt")
        self.slider_belt.setMaximum(450)
        self.slider_belt.setPageStep(50)
        self.slider_belt.setOrientation(Qt.Horizontal)
        self.slider_belt.setTickPosition(QSlider.NoTicks)
        self.slider_belt.setTickInterval(0)

        self.horizontalLayout.addWidget(self.slider_belt)

        self.spinBox_belt = QSpinBox(self.groupBox_5)
        self.spinBox_belt.setObjectName(u"spinBox_belt")
        self.spinBox_belt.setMaximum(450)

        self.horizontalLayout.addWidget(self.spinBox_belt)


        self.verticalLayout_2.addWidget(self.groupBox_5)

        self.groupBox_4 = QGroupBox(Form)
        self.groupBox_4.setObjectName(u"groupBox_4")

        self.verticalLayout_2.addWidget(self.groupBox_4)


        self.horizontalLayout_2.addLayout(self.verticalLayout_2)


        self.verticalLayout_3.addLayout(self.horizontalLayout_2)


        self.retranslateUi(Form)
        self.slider_belt.valueChanged.connect(self.spinBox_belt.setValue)
        self.spinBox_belt.valueChanged.connect(self.slider_belt.setValue)

        QMetaObject.connectSlotsByName(Form)
    # setupUi

    def retranslateUi(self, Form):
        Form.setWindowTitle(QCoreApplication.translate("Form", u"Robot Test", None))
        self.groupBox_6.setTitle(QCoreApplication.translate("Form", u"Serial received watch", None))
        self.groupBox_8.setTitle(QCoreApplication.translate("Form", u"Serial send watch", None))
        self.groupBox_7.setTitle(QCoreApplication.translate("Form", u"Auto align watch", None))
        self.groupBox_3.setTitle(QCoreApplication.translate("Form", u"Image view", None))
        self.label_image.setText("")
        self.radioButton_test.setText(QCoreApplication.translate("Form", u"Test frame", None))
        self.radioButton_color.setText(QCoreApplication.translate("Form", u"Color frame", None))
        self.radioButton_depth.setText(QCoreApplication.translate("Form", u"Depth frame", None))
        self.groupBox.setTitle(QCoreApplication.translate("Form", u"Move", None))
        self.pushButton_rleft.setText(QCoreApplication.translate("Form", u"Rotate left", None))
        self.pushButton_forward.setText(QCoreApplication.translate("Form", u"Forward", None))
        self.pushButton_rright.setText(QCoreApplication.translate("Form", u"Rotate right", None))
        self.pushButton_left.setText(QCoreApplication.translate("Form", u"Left", None))
        self.pushButton_back.setText(QCoreApplication.translate("Form", u"Back", None))
        self.pushButton_right.setText(QCoreApplication.translate("Form", u"Right", None))
        self.groupBox_2.setTitle(QCoreApplication.translate("Form", u"Action", None))
        self.pushButton_body_lift.setText(QCoreApplication.translate("Form", u"body_lift", None))
        self.pushButton_camera_lift.setText(QCoreApplication.translate("Form", u"camera_lift", None))
        self.pushButton_grasp.setText(QCoreApplication.translate("Form", u"grasp", None))
        self.pushButton_push.setText(QCoreApplication.translate("Form", u"push", None))
        self.pushButton_auto_align.setText(QCoreApplication.translate("Form", u"auto_align", None))
        self.pushButton_flip.setText(QCoreApplication.translate("Form", u"flip", None))
        self.groupBox_5.setTitle(QCoreApplication.translate("Form", u"Belt", None))
        self.groupBox_4.setTitle(QCoreApplication.translate("Form", u"Robot infomation", None))
    # retranslateUi


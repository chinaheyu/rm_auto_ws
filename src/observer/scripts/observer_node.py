#!/usr/bin/env python3
from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *
import sys
from gui_package.UI_gui import Ui_Form
from robot_package.robot import Robot
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from common.msg import serial_send_msg
from common.msg import serial_received_msg
from std_msgs.msg import Bool
import cv2

class GUI(QWidget):
    serial_send_signal = Signal(str)
    serial_received_signal = Signal(str)
    align_state_signal = Signal(str)
    def __init__(self):
        super(GUI, self).__init__()
        self.is_depth_frame = False
        self.robot = Robot('observer')
        self.image_sub = rospy.Subscriber('/camera/color/image_raw/compressed', CompressedImage, self.frame_received_callback, queue_size=1, tcp_nodelay=True)
        self.serial_send_sub = rospy.Subscriber('/serial_send_msg', serial_send_msg, self.serial_send_callback, queue_size=1)
        self.serial_received_sub = rospy.Subscriber('/serial_received_msg', serial_received_msg, self.serial_received_callback, queue_size=1)
        self.align_state_sub = rospy.Subscriber('/align_state', Bool, self.align_state_callback, queue_size=1)
        self.bridge = CvBridge()
        self.test_line_speed = 2.0
        self.test_rotate_speed = 6.0
        self.ui = Ui_Form()
        self.ui.setupUi(self)
        self.ui.pushButton_forward.pressed.connect(self.forward_pressed)
        self.ui.pushButton_forward.released.connect(self.forward_released)
        self.ui.pushButton_back.pressed.connect(self.back_pressed)
        self.ui.pushButton_back.released.connect(self.back_released)
        self.ui.pushButton_left.pressed.connect(self.left_pressed)
        self.ui.pushButton_left.released.connect(self.left_released)
        self.ui.pushButton_right.pressed.connect(self.right_pressed)
        self.ui.pushButton_right.released.connect(self.right_released)
        self.ui.pushButton_rleft.pressed.connect(self.rleft_pressed)
        self.ui.pushButton_rleft.released.connect(self.rleft_released)
        self.ui.pushButton_rright.pressed.connect(self.rright_pressed)
        self.ui.pushButton_rright.released.connect(self.rright_released)
        self.ui.pushButton_body_lift.clicked.connect(self.body_lift_clicked)
        self.ui.pushButton_grasp.clicked.connect(self.grasp_clicked)
        self.ui.pushButton_push.clicked.connect(self.push_clicked)
        self.ui.pushButton_camera_lift.clicked.connect(self.camera_lift_clicked)
        self.ui.pushButton_flip.clicked.connect(self.flip_clicked)
        self.ui.pushButton_auto_align.clicked.connect(self.auto_align_clicked)
        self.ui.slider_belt.valueChanged.connect(self.belt_value_changed)
        self.ui.radioButton_test.clicked.connect(self.test_checked)
        self.ui.radioButton_color.clicked.connect(self.color_checked)
        self.ui.radioButton_depth.clicked.connect(self.depth_checked)
        self.serial_send_signal.connect(self.ui.plainTextEdit_send.setPlainText)
        self.serial_received_signal.connect(self.ui.plainTextEdit_received.setPlainText)
        self.align_state_signal.connect(self.ui.plainTextEdit_align.setPlainText)
        self.tim1 = QTimer(self)
        self.tim1.timeout.connect(self.update_odom)
        self.tim1.setInterval(100)
        self.tim1.start()
        self.tim2 = QTimer(self)
        self.tim2.timeout.connect(self.shut_node)
        self.tim2.setInterval(200)
        self.tim2.start()

    @Slot()
    def shut_node(self):
        if self.robot.is_shutdown:
            self.close()

    @Slot()
    def update_odom(self):
        tmp = self.robot.odometry
        self.ui.label_x.setText('x: {:.3f}'.format(tmp['x']))
        self.ui.label_y.setText('y: {:.3f}'.format(tmp['y']))
        self.ui.label_w.setText('w: {:.3f}'.format(tmp['w']))
        self.ui.label_vx.setText('vx: {:.3f}'.format(tmp['vx']))
        self.ui.label_vy.setText('vy: {:.3f}'.format(tmp['vy']))
        self.ui.label_vw.setText('vw: {:.3f}'.format(tmp['vw']))

    def frame_received_callback(self, msg):
        if self.is_depth_frame:
            frame = self.bridge.imgmsg_to_cv2(msg)
            frame = cv2.applyColorMap(cv2.convertScaleAbs(frame, alpha=0.03), cv2.COLORMAP_JET)
            image = QImage(frame, 640, 480, frame.strides[0], QImage.Format_RGB888)
            self.ui.label_image.setPixmap(QPixmap.fromImage(image))
        else:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = QImage(frame, 640, 480, frame.strides[0], QImage.Format_RGB888)
            self.ui.label_image.setPixmap(QPixmap.fromImage(image))
        
    def serial_send_callback(self, msg):
        msg_tmp = 'sign: {}\nvx: {}\nvy: {}\nvw: {}\naction: {}\nbelt: {}'.format(msg.sign, msg.vx, msg.vy, msg.vw, msg.action, msg.belt)
        self.serial_send_signal.emit(msg_tmp)

    def serial_received_callback(self, msg):
        msg_tmp = 'sign: {}'.format(msg.sign)
        self.serial_received_signal.emit(msg_tmp)

    def align_state_callback(self, msg):
        msg_tmp = 'align state: {}'.format(msg.data)
        if msg.data == False:
            self.ui.radioButton_color.click()
        self.align_state_signal.emit(msg_tmp)

    @Slot()
    def test_checked(self):
        self.is_depth_frame = False
        self.image_sub.unregister()
        self.image_sub = rospy.Subscriber('/test_frame/compressed', CompressedImage, self.frame_received_callback, queue_size=1, tcp_nodelay=True)

    @Slot()
    def color_checked(self):
        self.is_depth_frame = False
        self.image_sub.unregister()
        self.image_sub = rospy.Subscriber('/camera/color/image_raw/compressed', CompressedImage, self.frame_received_callback, queue_size=1, tcp_nodelay=True)

    @Slot()
    def depth_checked(self):
        self.is_depth_frame = True
        self.image_sub.unregister()
        self.image_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.frame_received_callback, queue_size=1, tcp_nodelay=True)
    
    @Slot()
    def forward_pressed(self):
        self.robot.set_velocity(self.test_line_speed, 0, 0)

    @Slot()
    def forward_released(self):
        self.robot.set_velocity(0, 0, 0)

    @Slot()
    def back_pressed(self):
        self.robot.set_velocity(-self.test_line_speed, 0, 0)

    @Slot()
    def back_released(self):
        self.robot.set_velocity(0, 0, 0)

    @Slot()
    def left_pressed(self):
        self.robot.set_velocity(0, self.test_line_speed, 0)

    @Slot()
    def left_released(self):
        self.robot.set_velocity(0, 0, 0)

    @Slot()
    def right_pressed(self):
        self.robot.set_velocity(0, -self.test_line_speed, 0)

    @Slot()
    def right_released(self):
        self.robot.set_velocity(0, 0, 0)

    @Slot()
    def rleft_pressed(self):
        self.robot.set_velocity(0, 0, self.test_rotate_speed)

    @Slot()
    def rleft_released(self):
        self.robot.set_velocity(0, 0, 0)

    @Slot()
    def rright_pressed(self):
        self.robot.set_velocity(0, 0, -self.test_rotate_speed)

    @Slot()
    def rright_released(self):
        self.robot.set_velocity(0, 0, 0)

    @Slot()
    def body_lift_clicked(self):
        self.robot.raise_body()

    @Slot()
    def grasp_clicked(self):
        self.robot.grasp()

    @Slot()
    def push_clicked(self):
        self.robot.push()

    @Slot()
    def camera_lift_clicked(self):
        self.robot.raise_camera()

    @Slot()
    def flip_clicked(self):
        self.robot.flip()

    @Slot()
    def auto_align_clicked(self):
        self.robot.alignToOre()
        self.ui.radioButton_test.click()

    @Slot()
    def belt_value_changed(self, value):
        self.robot.move_belt(value)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    w = GUI()
    w.show()
    app.exec_()
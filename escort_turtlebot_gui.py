import os
os.environ["QT_QPA_PLATFORM"] = "xcb"

import sys
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from cv_bridge import CvBridge

import cv2

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *


# -------------------------------------------------
# ROS2 Node
# -------------------------------------------------

class EscortGuiNode(Node):

    def __init__(self, gui):

        super().__init__('escort_gui')

        self.gui = gui
        self.bridge = CvBridge()

        # camera subscriber
        self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.image_callback,
            10
        )

        # gesture subscriber
        self.create_subscription(
            String,
            '/gesture_state',
            self.gesture_callback,
            10
        )
        # mask subscriber
        self.create_subscription(
            String,
            '/mask_state',
            self.mask_callback,
            10
        )

        # cmd_vel publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    # -----------------------------
    # mask callback
    # -----------------------------
    def mask_callback(self, msg):
        mask_name = msg.data
        self.gui.update_mask_status(mask_name)


    def image_callback(self, msg):

        frame = self.bridge.compressed_imgmsg_to_cv2(msg,'bgr8')

        self.gui.update_image(frame)


    def gesture_callback(self,msg):

        gesture = msg.data

        twist = Twist()

        if gesture == "FORWARD":
            twist.linear.x = 0.05
            self.gui.update_gesture("Moving Forward")

        elif gesture == "LEFT":
            twist.angular.z = 0.3
            self.gui.update_gesture("Turning Left")

        elif gesture == "RIGHT":
            twist.angular.z = -0.3
            self.gui.update_gesture("Turning Right")

        elif gesture == "STOP":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.gui.update_gesture("Stopped")

        self.cmd_pub.publish(twist)


    def move_robot(self, linear, angular):

        twist = Twist()

        twist.linear.x = linear
        twist.angular.z = angular

        self.cmd_pub.publish(twist)


# -------------------------------------------------
# GUI
# -------------------------------------------------

class EscortGUI(QWidget):

    def __init__(self):

        super().__init__()

        self.setWindowTitle("Escort Turtlebot Control Panel")
        self.setGeometry(100,100,820,650)

        self.current_gesture = "NONE"

        # camera view
        self.image_label = QLabel()
        self.image_label.setFixedSize(640,480)

        # gesture label
        self.gesture_label = QLabel("Gesture : NONE")
        self.gesture_label.setFont(QFont("Arial",18))

        # robot status → 이제 mask 상태만
        self.status_label = QLabel("Mask Status : Unknown")
        self.status_label.setFont(QFont("Arial",14))

        # buttons
        self.btn_forward = QPushButton("Forward")
        self.btn_left = QPushButton("Left")
        self.btn_right = QPushButton("Right")
        self.btn_stop = QPushButton("STOP")

        # layout
        btn_layout = QHBoxLayout()
        btn_layout.addWidget(self.btn_left)
        btn_layout.addWidget(self.btn_forward)
        btn_layout.addWidget(self.btn_right)
        btn_layout.addWidget(self.btn_stop)

        layout = QVBoxLayout()

        layout.addWidget(self.image_label)
        layout.addWidget(self.gesture_label)
        layout.addWidget(self.status_label)
        layout.addLayout(btn_layout)

        self.setLayout(layout)


    def set_node(self, node):

        self.node = node

        self.btn_forward.clicked.connect(lambda: self.node.move_robot(0.05,0.0))
        self.btn_left.clicked.connect(lambda: self.node.move_robot(0.0,0.3))
        self.btn_right.clicked.connect(lambda: self.node.move_robot(0.0,-0.3))
        self.btn_stop.clicked.connect(lambda: self.node.move_robot(0.0,0.0))


    def update_image(self, frame):

        frame = cv2.resize(frame,(640,480))

        # -----------------------------
        # 화면에 Gesture 표시
        # -----------------------------

        if self.current_gesture == "FORWARD":
            color = (0,255,0)

        elif self.current_gesture == "LEFT":
            color = (255,0,0)

        elif self.current_gesture == "RIGHT":
            color = (0,0,255)

        else:
            color = (0,255,255)

        cv2.putText(
            frame,
            f"Gesture : {self.current_gesture}",
            (20,40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            color,
            3
        )

        # 프레임 테두리 표시
        cv2.rectangle(
            frame,
            (0,0),
            (639,479),
            color,
            4
        )

        rgb = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)

        h,w,ch = rgb.shape

        qt_image = QImage(
            rgb.data,
            w,
            h,
            ch*w,
            QImage.Format_RGB888
        )

        self.image_label.setPixmap(QPixmap.fromImage(qt_image))


    def update_gesture(self, gesture):

        self.current_gesture = gesture

        self.gesture_label.setText(f"Gesture : {gesture}")

    def update_mask_status(self, mask_name):
        if mask_name == "with_mask":
            self.status_label.setText("Mask Status : Verified")
        elif mask_name == "without_mask":
            self.status_label.setText("Mask Status : Not Wearing")
        else:
            self.status_label.setText("Mask Status : Unknown")
# -------------------------------------------------
# main
# -------------------------------------------------

def main():

    rclpy.init()

    app = QApplication(sys.argv)

    gui = EscortGUI()

    node = EscortGuiNode(gui)

    gui.set_node(node)

    gui.show()

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    timer.start(10)

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

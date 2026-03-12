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
import numpy as np
import mediapipe as mp
from ultralytics import YOLO

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *


# -------------------------------------------------
# YOLO + Gesture Node
# -------------------------------------------------
class EscortGestureMaskNode(Node):

    def __init__(self):

        self.frame_count = 0

        super().__init__('escort_gesture_mask')

        self.bridge = CvBridge()

        # publishers
        self.gesture_pub = self.create_publisher(String,'/gesture_state',10)
        self.mask_pub = self.create_publisher(String,'/mask_state',10)

        self.cmd_pub = self.create_publisher(Twist,'/TB3_1/cmd_vel',10)

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

        # YOLO
        self.model = YOLO("/home/ubuntu/robot_ws/src/escort_robot/last.pt")

        self.colors = {
            "Mask_VIP":(0,255,0),
            "Hand":(0,255,255),
            "Phone":(0,0,255)
        }

        # mediapipe
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7
        )

        # self.mp_draw = mp.solutions.drawing_utils

        self.mask_verified = False

        self.get_logger().info("Escort System Started")


    # -------------------------
    # finger count
    # -------------------------
    def count_fingers(self,hand):

        lm = hand.landmark
        fingers = 0

        finger_sets = [
            (2,3,4),
            (5,6,8),
            (9,10,12),
            (13,14,16),
            (17,18,20)
        ]

        for a,b,c in finger_sets:

            A = np.array([lm[a].x,lm[a].y,lm[a].z])
            B = np.array([lm[b].x,lm[b].y,lm[b].z])
            C = np.array([lm[c].x,lm[c].y,lm[c].z])

            BA = A-B
            BC = C-B

            cosine = np.dot(BA,BC)/(np.linalg.norm(BA)*np.linalg.norm(BC))
            angle = np.degrees(np.arccos(np.clip(cosine,-1,1)))

            if angle>160:
                fingers+=1

        return fingers


    # -------------------------
    # gesture classify
    # -------------------------
    def classify_gesture(self,f):

        if f==0: return "STOP"
        if f==5: return "FORWARD"
        if f==1: return "LEFT"
        if f==2: return "RIGHT"

        return "STOP"


    # -------------------------
    # camera callback
    # -------------------------
    def image_callback(self,msg):

        self.frame_count += 1
        if self.frame_count % 10 != 0:  # 10프레임마다 1번만 YOLO 실행 (약 0.3초 간격)
            return

        # if self.frame_count % 5 == 0:
            results = self.model(frame, imgsz=320, conf=0.5, verbose=False)
        else:
            results = []

        frame = self.bridge.compressed_imgmsg_to_cv2(msg,'bgr8')
        frame = cv2.flip(frame,1)
        frame = cv2.resize(frame,(320,240))

        gesture="STOP"

        results = self.model(frame, imgsz=320, conf=0.5, verbose=False)

        for box in results[0].boxes:

            cls_id=int(box.cls.cpu().numpy()[0])
            conf=float(box.conf.cpu().numpy()[0])
            name=self.model.names[cls_id]

            mask_msg=String()
            mask_msg.data=name
            self.mask_pub.publish(mask_msg)

            x1,y1,x2,y2=map(int,box.xyxy.cpu().numpy()[0])

            color=self.colors.get(name,(255,255,255))

            # cv2.rectangle(frame,(x1,y1),(x2,y2),color,2)

            cv2.putText(frame,f"{name} {conf:.2f}",
                        (x1,y1-10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,color,2)

            if name=="Mask_VIP":
                self.mask_verified=True

        # gesture detect
        if self.mask_verified:

            rgb=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)

            if self.frame_count % 10 == 0:
                result = self.hands.process(rgb)

            if result.multi_hand_landmarks:

                for hand in result.multi_hand_landmarks:

                    # self.mp_draw.draw_landmarks(
                    #     frame,
                    #     hand,
                    #     self.mp_hands.HAND_CONNECTIONS
                    # )

                    f=self.count_fingers(hand)

                    gesture=self.classify_gesture(f)

                    g_msg=String()
                    g_msg.data=gesture

                    self.gesture_pub.publish(g_msg)

        # GUI 업데이트
        if hasattr(self,"gui"):
            self.gui.update_image(frame)


    # -------------------------
    # gesture callback
    # -------------------------
    def gesture_callback(self,msg):

        gesture=msg.data

        if hasattr(self,"gui"):
            self.gui.update_gesture(gesture)

        twist=Twist()

        if gesture=="FORWARD":
            twist.linear.x=0.05

        elif gesture=="LEFT":
            twist.angular.z=0.3

        elif gesture=="RIGHT":
            twist.angular.z=-0.3

        elif gesture=="STOP":
            twist.linear.x=0.0
            twist.angular.z=0.0

        self.cmd_pub.publish(twist)


# -------------------------------------------------
# GUI
# -------------------------------------------------
class EscortGUI(QWidget):

    def __init__(self):

        super().__init__()

        self.setWindowTitle("Escort Turtlebot Control Panel")
        self.setGeometry(100,100,820,650)

        self.current_gesture="NONE"
        self.current_status="Unknown"

        self.image_label=QLabel()
        self.image_label.setFixedSize(640,480)

        self.gesture_label=QLabel("Gesture : NONE")
        self.gesture_label.setFont(QFont("Arial",18))

        self.status_label=QLabel("Status : Unknown")
        self.status_label.setFont(QFont("Arial",14))

        self.btn_forward=QPushButton("Forward")
        self.btn_left=QPushButton("Left")
        self.btn_right=QPushButton("Right")
        self.btn_stop=QPushButton("STOP")

        btn_layout=QHBoxLayout()
        btn_layout.addWidget(self.btn_left)
        btn_layout.addWidget(self.btn_forward)
        btn_layout.addWidget(self.btn_right)
        btn_layout.addWidget(self.btn_stop)

        layout=QVBoxLayout()
        layout.addWidget(self.image_label)
        layout.addWidget(self.gesture_label)
        layout.addWidget(self.status_label)
        layout.addLayout(btn_layout)

        self.setLayout(layout)


    def set_node(self,node):

        self.node=node
        node.gui=self

        self.btn_forward.clicked.connect(lambda:self.node.cmd_pub.publish(Twist()))
        self.btn_stop.clicked.connect(lambda:self.node.cmd_pub.publish(Twist()))


    def update_image(self,frame):

        frame=cv2.resize(frame,(640,480))

        color_map={
            "FORWARD":(0,255,0),
            "LEFT":(255,0,0),
            "RIGHT":(0,0,255),
            "STOP":(0,255,255),
            "NONE":(0,255,255)
        }

        color=color_map.get(self.current_gesture,(0,255,255))

        cv2.putText(frame,
                    f"Gesture : {self.current_gesture}",
                    (20,40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,color,3)

        rgb=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)

        h,w,ch=rgb.shape

        img=QImage(rgb.data,w,h,ch*w,QImage.Format_RGB888)

        self.image_label.setPixmap(QPixmap.fromImage(img))


    def update_gesture(self,g):

        self.current_gesture=g
        self.gesture_label.setText(f"Gesture : {g}")


# -------------------------------------------------
# main
# -------------------------------------------------
def main():

    rclpy.init()

    app=QApplication(sys.argv)

    gui=EscortGUI()

    node=EscortGestureMaskNode()

    gui.set_node(node)

    gui.show()

    timer=QTimer()
    timer.timeout.connect(lambda:rclpy.spin_once(node,timeout_sec=0))
    timer.start(10)

    sys.exit(app.exec_())


if __name__=="__main__":
    main()

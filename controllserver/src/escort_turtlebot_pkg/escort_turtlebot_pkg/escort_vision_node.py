import os
import sys

# Workaround for Qt conflict: Load system Qt plugins
os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = "/usr/lib/x86_64-linux-gnu/qt5/plugins/"

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import String
import numpy as np

import cv2
import mediapipe as mp
from ultralytics import YOLO


class EscortGestureMaskNode(Node):

    def __init__(self):
        super().__init__('escort_gesture_mask')

        self.bridge = CvBridge()

        self.gesture_pub = self.create_publisher(String, '/gesture_state', 10)
        self.mask_pub = self.create_publisher(String,'/mask_state',10)

        # Turtlebot cmd_vel publisher
        #self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # camera subscriber
        self.sub = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.image_callback,
            10
        )

        # -----------------------------
        # YOLO mask model
        # -----------------------------
        self.declare_parameter('model_path', os.path.join(os.path.expanduser('~'), 'escort_ws/team_project/best.pt'))
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.get_logger().info(f"Loading YOLO model from: {model_path}")
        self.model = YOLO(model_path)

        self.colors = {
            "with_mask": (0,255,0),
            "without_mask": (0,0,255),
            "mask_weared_incorrect": (0,255,255)
        }

        # -----------------------------
        # Mediapipe hands
        # -----------------------------
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7
        )

        self.mp_draw = mp.solutions.drawing_utils

        # -----------------------------
        # 상태 변수
        # -----------------------------
        self.mask_verified = False

        self.get_logger().info("Escort Turtlebot System Started")

    # -------------------------
    # 각도 계산
    # -------------------------
    def calculate_angle(self, a, b, c):

        a = np.array(a)
        b = np.array(b)
        c = np.array(c)

        ba = a - b
        bc = c - b

        cosine = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
        angle = np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0)))

        return angle

    # -------------------------
    # 손가락 개수 계산 (각도 기반)
    # -------------------------
    def count_fingers(self, hand_landmarks):

        lm = hand_landmarks.landmark

        fingers = 0

        finger_sets = [
        (2,3,4),     # thumb
        (5,6,8),     # index
        (9,10,12),   # middle
        (13,14,16),  # ring
        (17,18,20)   # pinky
    ]

        for a,b,c in finger_sets:

            A = np.array([lm[a].x, lm[a].y, lm[a].z])
            B = np.array([lm[b].x, lm[b].y, lm[b].z])
            C = np.array([lm[c].x, lm[c].y, lm[c].z])

            BA = A - B
            BC = C - B

            cosine = np.dot(BA, BC) / (np.linalg.norm(BA) * np.linalg.norm(BC))
            angle = np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0)))

            if angle > 160:
                fingers += 1

        return fingers


    # -------------------------------------------------
    # 제스처 분류
    # -------------------------------------------------
    def classify_gesture(self, finger_count):

        if finger_count == 0:
            return "STOP"

        if finger_count == 5:
            return "FORWARD"

        if finger_count == 1:
            return "LEFT"

        if finger_count == 2:
            return "RIGHT"

        return "STOP"


    # -------------------------------------------------
    # 카메라 콜백
    # -------------------------------------------------
    def image_callback(self, msg):
        if not msg or not msg.data:
            return

        try:
            # Decode compressed image
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if frame is None:
                return

            frame = cv2.flip(frame, 1)
            frame = cv2.resize(frame, (320, 240))
        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")
            return

        gesture = "STOP"

        # -------------------------------------------------
        # 1️⃣ 아직 마스크 확인 안됐으면 YOLO 실행
        # -------------------------------------------------
        if not self.mask_verified:

            results = self.model(frame, conf=0.1)

            for box in results[0].boxes:

                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                name = self.model.names[cls_id]

                mask_msg = String()
                mask_msg.data = name
                self.mask_pub.publish(mask_msg)

                x1,y1,x2,y2 = map(int,box.xyxy[0])

                color = self.colors.get(name,(255,255,255))

                cv2.rectangle(frame,(x1,y1),(x2,y2),color,2)

                cv2.putText(
                    frame,
                    f"{name} {conf:.2f}",
                    (x1,y1-10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    color,
                    2
                )

                if name == "with_mask":
                    self.mask_verified = True
                    self.get_logger().info("Mask verified. Gesture control enabled.")

            if not self.mask_verified:

                cv2.putText(
                    frame,
                    "PLEASE WEAR A MASK",
                    (40,120),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (0,0,255),
                    3
                )

        # -------------------------------------------------
        # 2️⃣ 마스크 확인 후 → 제스처 인식
        # -------------------------------------------------
        else:

            rgb = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
            result = self.hands.process(rgb)

            if result.multi_hand_landmarks:

                for hand_landmarks in result.multi_hand_landmarks:

                    self.mp_draw.draw_landmarks(
                        frame,
                        hand_landmarks,
                        self.mp_hands.HAND_CONNECTIONS
                    )

                    finger_count = self.count_fingers(hand_landmarks)

                    gesture = self.classify_gesture(finger_count)

                    gesture_msg = String()
                    gesture_msg.data = gesture
                    self.gesture_pub.publish(gesture_msg)

        # -------------------------------------------------
        # 3️⃣ 로봇 제어
        # -------------------------------------------------
        # twist = Twist()

        # if gesture == "FORWARD":
        #     twist.linear.x = 0.05

        # elif gesture == "LEFT":
        #     twist.angular.z = 0.3

        # elif gesture == "RIGHT":
        #     twist.angular.z = -0.3

        # elif gesture == "STOP":
        #     twist.linear.x = 0.0
        #     twist.angular.z = 0.0

        # self.cmd_pub.publish(twist)

        # -------------------------------------------------
        # 화면 표시
        # -------------------------------------------------
        cv2.putText(
            frame,
            f"Gesture: {gesture}",
            (10,30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0,255,0),
            2
        )

        if self.mask_verified:
            cv2.putText(
                frame,
                "MASK VERIFIED",
                (160,30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0,255,0),
                2
            )

        # cv2.imshow("Escort Turtlebot Vision",frame)
        # cv2.waitKey(1)


# -------------------------------------------------
# main
# -------------------------------------------------
def main(args=None):

    rclpy.init(args=args)

    node = EscortGestureMaskNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

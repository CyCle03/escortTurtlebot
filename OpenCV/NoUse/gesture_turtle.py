import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist

import cv2
import mediapipe as mp
import numpy as np
from collections import deque


class GestureTurtle(Node):

    def __init__(self):
        super().__init__('gesture_turtle')

        # camera subscriber
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.image_callback,
            qos_profile_sensor_data
        )

        # robot command publisher
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # mediapipe
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            max_num_hands=2,
            min_detection_confidence=0.85,
            min_tracking_confidence=0.85,
            model_complexity=1
        )

        self.mp_draw = mp.solutions.drawing_utils

        # gesture smoothing buffer
        self.gesture_buffer = deque(maxlen=5)

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
    # -------------------------
    # 제스처 결정
    # -------------------------
    def classify_gesture(self, finger_count):

        if finger_count == 5:
            return "FORWARD"

        elif finger_count == 0:
            return "STOP"

        elif finger_count == 1:
            return "LEFT"

        elif finger_count == 2:
            return "RIGHT"

        return "NONE"

    # -------------------------
    # image callback
    # -------------------------
    def image_callback(self, msg):

        # compressed image → OpenCV
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        frame = cv2.flip(frame, 1)

        # 연산 속도 개선
        frame = cv2.resize(frame, (320, 240))

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        result = self.hands.process(rgb)

        gesture = "NONE"

        if result.multi_hand_landmarks:

            for hand_landmarks in result.multi_hand_landmarks:

                self.mp_draw.draw_landmarks(
                    frame,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS
                )

                finger_count = self.count_fingers(hand_landmarks)

                gesture = self.classify_gesture(finger_count)

        else:
            gesture = "STOP"

        # -------------------------
        # 제스처 smoothing
        # -------------------------
        self.gesture_buffer.append(gesture)

        gesture = max(set(self.gesture_buffer),
                      key=self.gesture_buffer.count)

        # -------------------------
        # robot control
        # -------------------------
        twist = Twist()

        if gesture == "FORWARD":

            twist.linear.x = 0.05

        elif gesture == "LEFT":

            twist.angular.z = 1.0

        elif gesture == "RIGHT":

            twist.angular.z = -1.0

        elif gesture == "STOP":

            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

        # 화면 표시
        cv2.putText(frame,
                    gesture,
                    (30, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0,255,0),
                    2)

        cv2.imshow("gesture control", frame)
        cv2.waitKey(1)


def main():

    rclpy.init()

    node = GestureTurtle()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

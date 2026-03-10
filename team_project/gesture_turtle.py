import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import mediapipe as mp
import numpy as np
from collections import deque

class GestureTurtle(Node):
    def __init__(self, node_name='gesture_turtle'):
        super().__init__(node_name)

        # ---------------------------------
        # 캠으로 카메라 직접 열기
        # ---------------------------------
        self.cap = cv2.VideoCapture(0)
        
        # 리더 로봇 전용 토픽으로 변경 (리스트에서 확인한 이름)
        self.cmd_pub = self.create_publisher(Twist, '/leader/cmd_vel', 10)

        # Mediapipe 설정
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.85,
            min_tracking_confidence=0.85,
            model_complexity=1
        )
        self.mp_draw = mp.solutions.drawing_utils
        self.gesture_buffer = deque(maxlen=5)

        # 30fps 주기로 처리
        self.timer = self.create_timer(0.033, self.process_webcam)

    # -------------------------
    # 원본 각도 계산 로직 (유지)
    # -------------------------
    def calculate_angle(self, a, b, c):
        a = np.array(a)
        b = np.array(b)
        c = np.array(c)
        ba = a - b
        bc = c - b # 여기서 bc는 내부 변수이므로 소문자 OK
        cosine = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc) + 1e-6)
        angle = np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0)))
        return angle

    # -------------------------
    # 원본 손가락 개수 계산 (유지)
    # -------------------------
    def count_fingers(self, hand_landmarks):
        lm = hand_landmarks.landmark
        fingers = 0
        finger_sets = [
            (2,3,4), (5,6,8), (9,10,12), (13,14,16), (17,18,20)
        ]

        for a, b, c in finger_sets:
            A = [lm[a].x, lm[a].y, lm[a].z]
            B = [lm[b].x, lm[b].y, lm[b].z]
            C = [lm[c].x, lm[c].y, lm[c].z]

            # 원본 함수 호출
            angle = self.calculate_angle(A, B, C)

            if angle > 160:
                fingers += 1
        return fingers

    def classify_gesture(self, finger_count):
        if finger_count == 5: return "FORWARD"
        elif finger_count == 0: return "STOP"
        elif finger_count == 1: return "LEFT"
        elif finger_count == 2: return "RIGHT"
        return "NONE"

    def process_webcam(self):
        ret, frame = self.cap.read()
        if not ret: return

        frame = cv2.flip(frame, 1)
        frame = cv2.resize(frame, (320, 240))
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.hands.process(rgb)

        gesture = "STOP"
        if result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                finger_count = self.count_fingers(hand_landmarks)
                gesture = self.classify_gesture(finger_count)

        self.gesture_buffer.append(gesture)
        smoothed_gesture = max(set(self.gesture_buffer), key=self.gesture_buffer.count)

        twist = Twist()
        if smoothed_gesture == "FORWARD": twist.linear.x = 0.15
        elif smoothed_gesture == "LEFT": twist.angular.z = 0.5
        elif smoothed_gesture == "RIGHT": twist.angular.z = -0.5
        
        self.cmd_pub.publish(twist)

        cv2.putText(frame, smoothed_gesture, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("Gesture Control", frame)
        cv2.waitKey(1)

    def __del__(self):
        if hasattr(self, 'cap'):
            self.cap.release()
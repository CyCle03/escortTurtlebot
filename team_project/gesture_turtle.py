import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import cv2
import mediapipe as mp
from ultralytics import YOLO

class EscortGestureMaskNode(Node):

    def __init__(self):
        super().__init__('escort_gesture_mask')

        # 1. 웹캠 초기화 (0번 카메라)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("카메라를 열 수 없습니다!")
            exit()

        # 2. 퍼블리셔 설정
        self.gesture_pub = self.create_publisher(String, '/gesture_state', 10)
        self.mask_pub = self.create_publisher(String, '/mask_state', 10)
        self.cmd_pub = self.create_publisher(Twist, '/leader/cmd_vel', 10) # 제어용 추가

        # 3. 타이머 설정 (30FPS 타겟)
        self.timer = self.create_timer(0.033, self.process_frame)

        # YOLO 마스크 모델 설정
        self.model = YOLO("/home/robot/robot_ws/src/team_project/team_project/best.pt")
        self.colors = {
            "with_mask": (0, 255, 0),
            "without_mask": (0, 0, 255),
            "mask_weared_incorrect": (0, 255, 255)
        }

        # Mediapipe hands 설정
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7
        )
        self.mp_draw = mp.solutions.drawing_utils

        # 상태 변수
        self.mask_verified = False
        self.get_logger().info("Webcam-based Escort System Started")

    def count_fingers(self, hand_landmarks):
        lm = hand_landmarks.landmark
        fingers = 0
        finger_sets = [(2,3,4), (5,6,8), (9,10,12), (13,14,16), (17,18,20)]
        for a,b,c in finger_sets:
            A = np.array([lm[a].x, lm[a].y, lm[a].z])
            B = np.array([lm[b].x, lm[b].y, lm[b].z])
            C = np.array([lm[c].x, lm[c].y, lm[c].z])
            BA = A - B
            BC = C - B
            cosine = np.dot(BA, BC) / (np.linalg.norm(BA) * np.linalg.norm(BC))
            angle = np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0)))
            if angle > 160: fingers += 1
        return fingers

    def classify_gesture(self, finger_count):
        if finger_count == 0: return "STOP"
        if finger_count == 5: return "FORWARD"
        if finger_count == 1: return "LEFT"
        if finger_count == 2: return "RIGHT"
        return "STOP"

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret: return

        frame = cv2.flip(frame, 1)
        frame = cv2.resize(frame, (640, 480)) # 가독성을 위해 해상도 상향
        gesture = "STOP"

        # 1️⃣ 마스크 미인증 시 YOLO 실행
        if not self.mask_verified:
            
            results = self.model(frame, stream=True, conf=0.965, verbose=False)

            # 제너레이터이므로 for문을 통해 결과 객체를 하나씩 가져옵니다.
            for result in results:
                # 각 결과 객체(result) 안에 있는 박스들을 확인
                for box in result.boxes:
                    cls_id = int(box.cls[0])
                    name = self.model.names[cls_id]
                    
                    # 마스크 상태 퍼블리시
                    mask_msg = String()
                    mask_msg.data = name
                    self.mask_pub.publish(mask_msg)

                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    color = self.colors.get(name, (255, 255, 255))
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(frame, name, (x1, y1-10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                    if name == "with_mask":
                        self.mask_verified = True
                        self.get_logger().info("Mask verified! Control enabled.")

            if not self.mask_verified:
                cv2.putText(frame, "PLEASE WEAR A MASK", (100, 240), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,0,255), 3)

        # 2️⃣ 마스크 인증 완료 후 제스처 인식
        else:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            result = self.hands.process(rgb)
            if result.multi_hand_landmarks:
                for hand_landmarks in result.multi_hand_landmarks:
                    self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                    finger_count = self.count_fingers(hand_landmarks)
                    gesture = self.classify_gesture(finger_count)
                    
                    gesture_msg = String()
                    gesture_msg.data = gesture
                    self.gesture_pub.publish(gesture_msg)

        # 3️⃣ 로봇 제어 (실제 이동을 위해 Twist 발행)
        twist = Twist()
        if gesture == "FORWARD": twist.linear.x = 0.1
        elif gesture == "LEFT": twist.angular.z = 0.5
        elif gesture == "RIGHT": twist.angular.z = -0.5
        self.cmd_pub.publish(twist)

        # 화면 UI 표시
        cv2.putText(frame, f"Gesture: {gesture}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        if self.mask_verified:
            cv2.putText(frame, "MASK VERIFIED", (400, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        cv2.imshow("Escort Vision System", frame)
        cv2.waitKey(1)

# 파일 하단 예시
def main(args=None):  # <-- 이 이름이 중요합니다!
    rclpy.init(args=args)
    node = EscortGestureMaskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

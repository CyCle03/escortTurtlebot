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

        # 1. 웹캠 초기화 (시스템의 0번 카메라)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("카메라를 열 수 없습니다! 연결을 확인하세요.")
            exit()

        # 2. 퍼블리셔 설정
        self.gesture_pub = self.create_publisher(String, '/gesture_state', 10)
        self.mask_pub = self.create_publisher(String, '/mask_state', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10) # 터틀봇 제어 토픽

        # 3. 타이머 설정 (약 30FPS)
        self.timer = self.create_timer(0.033, self.process_frame)

        # YOLO 마스크 모델 설정 (경로 확인 필수)
        # 본인의 환경에 맞는 절대 경로로 수정하세요.
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
        self.get_logger().info("Webcam-based Escort System Started (Conf: 0.1)")

    def count_fingers(self, hand_landmarks):
        lm = hand_landmarks.landmark
        fingers = 0
        # 엄지부터 소지까지 각도 계산을 위한 포인트 세트
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

        frame = cv2.flip(frame, 1) # 좌우 반전
        frame = cv2.resize(frame, (640, 480)) 
        gesture = "STOP"

        # -------------------------------------------------
        # 1️⃣ 마스크 인증 전: YOLO 탐지 (conf=0.1)
        # -------------------------------------------------
        if not self.mask_verified:
            # stream=False(기본값)이므로 리스트로 결과 반환
            results = self.model(frame, conf=0.1, verbose=False)
            
            # 첫 번째 결과 객체 가져오기
            result = results[0]
            
            for box in result.boxes:
                cls_id = int(box.cls[0])
                name = self.model.names[cls_id]
                conf_val = float(box.conf[0])
                
                # 마스크 상태 퍼블리시
                mask_msg = String()
                mask_msg.data = name
                self.mask_pub.publish(mask_msg)

                # 바운딩 박스 시각화
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                color = self.colors.get(name, (255, 255, 255))
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, f"{name} {conf_val:.2f}", (x1, y1-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                # 마스크 착용 확인 시 플래그 변경
                if name == "with_mask":
                    self.mask_verified = True
                    self.get_logger().info(f"Mask verified! Conf: {conf_val:.2f}")

            if not self.mask_verified:
                cv2.putText(frame, "PLEASE WEAR A MASK", (100, 240), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,0,255), 3)

        # -------------------------------------------------
        # 2️⃣ 마스크 인증 후: Mediapipe 손 제스처 인식
        # -------------------------------------------------
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

        # -------------------------------------------------
        # 3️⃣ 로봇 제어 (Twist 발행)
        # -------------------------------------------------
        twist = Twist()
        if gesture == "FORWARD": twist.linear.x = 0.08  # 속도는 환경에 맞춰 조절
        elif gesture == "LEFT": twist.angular.z = 0.4
        elif gesture == "RIGHT": twist.angular.z = -0.4
        else: # STOP 또는 인식 불능
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
        self.cmd_pub.publish(twist)

        # 화면 UI 표시
        cv2.putText(frame, f"Gesture: {gesture}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        if self.mask_verified:
            cv2.putText(frame, "MASK VERIFIED", (400, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow("Escort Vision System", frame)
        cv2.waitKey(1)

def main(args=None):
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
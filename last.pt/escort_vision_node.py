import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge

import cv2
import numpy as np
import mediapipe as mp
from ultralytics import YOLO


class EscortVisionNode(Node):

    def __init__(self):

        super().__init__('escort_vision_node')

        self.bridge = CvBridge()

        # publishers
        self.mask_pub = self.create_publisher(String,'/mask_state',10)
        self.gesture_pub = self.create_publisher(String,'/gesture_state',10)

        # camera subscriber
        self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.image_callback,
            10
        )

        # YOLO model
        self.model = YOLO("/home/ubuntu/robot_ws/src/escort_robot/last.pt")

        # mediapipe
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7
        )

        self.mask_verified = False

        self.get_logger().info("Escort Vision Node Started")


    def classify_gesture(self,fingers):

        if fingers == 0:
            return "STOP"

        elif fingers == 5:
            return "FORWARD"

        elif fingers == 1:
            return "LEFT"

        elif fingers == 2:
            return "RIGHT"

        return "STOP"


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

            if angle > 160:
                fingers += 1

        return fingers


    def image_callback(self,msg):

        frame = self.bridge.compressed_imgmsg_to_cv2(msg,'bgr8')
        frame = cv2.resize(frame,(320,240))

        results = self.model(frame)

        for box in results[0].boxes:

            cls_id = int(box.cls.cpu().numpy()[0])
            name = self.model.names[cls_id]

            mask_msg = String()
            mask_msg.data = name
            self.mask_pub.publish(mask_msg)

            if name == "Mask_VIP":
                self.mask_verified = True

        # gesture detection
        if self.mask_verified:

            rgb = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
            result = self.hands.process(rgb)

            if result.multi_hand_landmarks:

                for hand in result.multi_hand_landmarks:

                    fingers = self.count_fingers(hand)

                    gesture = self.classify_gesture(fingers)

                    msg = String()
                    msg.data = gesture

                    self.gesture_pub.publish(msg)


def main():

    rclpy.init()

    node = EscortVisionNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

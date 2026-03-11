import rclpy
from team_project.gesture_turtle import EscortGestureMaskNode
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import cv2

class LeaderNode(EscortGestureMaskNode):
    def __init__(self):
        super().__init__()
        # 리더의 현재 odom 위치 구독
        self.create_subscription(Odometry, '/leader/odom', self.odom_callback, 10)
        # 리더의 위치 좌표를 발행할 토픽
        self.pos_pub = self.create_publisher(Point, '/leader/path_pos', 10)
        self.get_logger().info('Leader Node 가동 중... 경로 데이터 발행 시작')

    def odom_callback(self, msg):
        # 리더의 현재 위치(x, y)를 그대로 전송
        target = Point()
        target.x = msg.pose.pose.position.x
        target.y = msg.pose.pose.position.y
        target.z = 0.0
        self.pos_pub.publish(target)

def main(args=None):
    rclpy.init(args=args)
    node = LeaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'cap'):
            node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
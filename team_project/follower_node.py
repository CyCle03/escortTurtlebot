import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import math
from collections import deque

class FollowerNode(Node):
    def __init__(self):
        super().__init__('follower_node')
        self.cmd_pub = self.create_publisher(Twist, '/follower/cmd_vel', 10)
        self.create_subscription(Odometry, '/follower/odom', self.odom_callback, 10)
        self.create_subscription(Point, '/leader/path_pos', self.target_callback, 10)
        
        # 상태 변수
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_yaw = 0.0
        
        # 경로 저장용 큐 및 설정
        self.path_queue = deque()
        self.min_follow_dist = 2.0  # 추종 시작 거리 (2미터)
        self.stop_dist = 0.1       # 목표 도달 판정 거리
        
        self.get_logger().info(f'Follower Node 가동... 설정 거리: {self.min_follow_dist}m')

    def odom_callback(self, msg):
        self.curr_x = msg.pose.pose.position.x
        self.curr_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # 쿼터니언 -> Yaw 변환
        self.curr_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))

    def target_callback(self, msg):
        # 1. 수신된 리더의 위치를 큐에 추가
        self.path_queue.append((msg.x, msg.y))
        
        # 2. 리더와 팔로워 사이의 현재 직선 거리 계산
        dist_to_leader = math.sqrt((msg.x - self.curr_x)**2 + (msg.y - self.curr_y)**2)

        twist = Twist()

        # 3. 거리가 2m 이상일 때만 큐에서 좌표를 꺼내 이동
        if dist_to_leader >= self.min_follow_dist and self.path_queue:
            # 큐의 가장 오래된 좌표(리더의 과거 위치)를 목표로 설정
            target_x, target_y = self.path_queue.popleft()
            
            dx = target_x - self.curr_x
            dy = target_y - self.curr_y
            dist_to_target = math.sqrt(dx**2 + dy**2)
            
            if dist_to_target > self.stop_dist:
                angle_to_target = math.atan2(dy, dx)
                angle_error = math.atan2(math.sin(angle_to_target - self.curr_yaw), 
                                         math.cos(angle_to_target - self.curr_yaw))

                # 속도 제어 (P-제어)
                twist.linear.x = min(0.6 * dist_to_target, 0.22)
                twist.angular.z = 2.0 * angle_error
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
        else:
            # 거리가 가깝거나 큐가 비어있으면 정지
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = FollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist()) # 정지 명령
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
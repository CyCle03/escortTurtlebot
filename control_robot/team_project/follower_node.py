import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import math
from collections import deque

class FollowerNode(Node):
    def __init__(self):
        super().__init__('follower_node')
        
        # 터틀봇 모터 제어 퍼블리셔 (올려주신 토픽명 유지)
        # self.cmd_pub = self.create_publisher(Twist, '/TB3_2/cmd_vel', 10)
        # self.create_subscription(Odometry, '/TB3_2/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Point, '/leader/path_pos', self.target_callback, 10)
        self.create_subscription(Float32, '/ultrasonic_distance', self.sonar_callback, 10)
        
        # 상태 변수
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_yaw = 0.0
        self.sonar_dist = 999.0
        
        self.path_queue = deque()
        self.min_follow_dist = 0.5   # 리더와 유지할 거리 (필요시 조절)
        self.stop_dist = 0.1
        self.current_target = None

        self.get_logger().info(' 추적 알고리즘 + 초음파')

    def odom_callback(self, msg):
        self.curr_x = msg.pose.pose.position.x
        self.curr_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # Quaternion to Yaw 변환
        self.curr_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))

    def sonar_callback(self, msg):
        self.sonar_dist = msg.data
        # 초음파 데이터가 들어올 때도 제어 루프를 돌려야 반응이 빠릅니다.
        self.control_loop()

    def target_callback(self, msg):
        self.path_queue.append((msg.x, msg.y))
        if self.current_target is None and self.path_queue:
            self.current_target = self.path_queue.popleft()
        self.control_loop()

    def control_loop(self):
        twist = Twist()

        # 1순위: 초음파 긴급 회피 (10cm 이하)
        if self.sonar_dist <= 10.0:
            self.get_logger().warn(f'긴급 후진! ({self.sonar_dist:.1f}cm)')
            twist.linear.x = -0.07  # 안정적인 후진 속도
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            return

        # 2순위: 리더 추적 알고리즘
        if self.current_target:
            tx, ty = self.current_target
            dx = tx - self.curr_x
            dy = ty - self.curr_y
            dist_to_target = math.sqrt(dx**2 + dy**2)

            # 목표물과의 거리가 유지 거리보다 멀 때만 이동
            if dist_to_target > self.min_follow_dist:
                angle_to_target = math.atan2(dy, dx)
                angle_error = math.atan2(math.sin(angle_to_target - self.curr_yaw), 
                                        math.cos(angle_to_target - self.curr_yaw))

                # P-제어 (거리 차이에 따른 속도 조절)
                error_dist = dist_to_target - self.min_follow_dist
                base_speed = 0.6 * error_dist 
                
                twist.linear.x = min(max(base_speed, 0.07), 0.20) # 속도 제한
                twist.angular.z = 2.0 * angle_error 
            
            # 목표 도달 시 다음 목표로 갱신
            if dist_to_target < self.stop_dist:
                if self.path_queue:
                    self.current_target = self.path_queue.popleft()
                else:
                    self.current_target = None
        
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = FollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    
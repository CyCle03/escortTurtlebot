import rclpy
from team_project.gesture_turtle import GestureTurtle
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import math
import cv2

class LeaderNode(GestureTurtle):
    def __init__(self):
        super().__init__(node_name='leader_node')
        self.follow_distance = 3.0
        
        self.create_subscription(Odometry, '/leader/odom', self.odom_callback, 10)
        self.pos_pub = self.create_publisher(Point, '/leader/relative_pos', 10)
        self.get_logger().info(f'Leader Node 가동 중... 추종 거리: {self.follow_distance}m')

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        
        target = Point()
        target.x = x - self.follow_distance * math.cos(yaw)
        target.y = y - self.follow_distance * math.sin(yaw)
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
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
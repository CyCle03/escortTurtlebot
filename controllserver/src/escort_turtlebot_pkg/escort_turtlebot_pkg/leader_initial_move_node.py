import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


class LeaderInitialMoveNode(Node):

    def __init__(self):
        super().__init__('leader_initial_move_node')
        self.declare_parameter('cmd_vel_topic', '/TB3_1/cmd_vel')
        self.declare_parameter('odom_topic', '/TB3_1/odom')
        self.declare_parameter('distance', 0.5)
        self.declare_parameter('speed', 0.10)
        self.declare_parameter('startup_delay_sec', 2.0)
        self.declare_parameter('max_duration_sec', 10.0)

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        distance = self.get_parameter('distance').get_parameter_value().double_value
        speed = self.get_parameter('speed').get_parameter_value().double_value
        startup_delay_sec = self.get_parameter(
            'startup_delay_sec').get_parameter_value().double_value
        self.max_duration_sec = self.get_parameter(
            'max_duration_sec').get_parameter_value().double_value

        self.speed = max(0.01, abs(speed))
        self.target_distance = abs(distance)
        self.direction = 1.0 if distance >= 0.0 else -1.0
        self.active = False
        self.completed = False
        self.elapsed = 0.0
        self.motion_started = False
        self.current_x = None
        self.current_y = None
        self.start_x = None
        self.start_y = None

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self._odom_callback, 10)

        self.timer_period = 0.05
        self.control_timer = self.create_timer(self.timer_period, self._control_loop)
        self.start_timer = self.create_timer(startup_delay_sec, self._start_move)

    def _odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def _start_move(self):
        self.start_timer.cancel()
        if self.target_distance <= 0.0:
            self.completed = True
            return
        self.active = True
        self.elapsed = 0.0
        self.start_x = None
        self.start_y = None

    def _control_loop(self):
        if self.completed:
            return

        if not self.active:
            # Wait until startup delay elapses.
            return

        # Wait for a live subscriber and odom before starting actual motion.
        if self.cmd_pub.get_subscription_count() < 1:
            return
        if self.current_x is None:
            return

        # Capture start pose once when motion really starts.
        if not self.motion_started:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.motion_started = True

        moved_distance = 0.0
        if self.motion_started:
            moved_distance = math.hypot(self.current_x - self.start_x,
                                        self.current_y - self.start_y)

        self.elapsed += self.timer_period

        twist = Twist()
        if moved_distance < self.target_distance and self.elapsed < self.max_duration_sec:
            twist.linear.x = self.direction * self.speed
        else:
            self.active = False
            self.completed = True
            self.control_timer.cancel()
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = LeaderInitialMoveNode()
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

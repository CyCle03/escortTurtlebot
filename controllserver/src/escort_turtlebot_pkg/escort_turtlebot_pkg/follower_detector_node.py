#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped, PointStamped
import tf2_ros
import tf2_geometry_msgs
import math

class FollowerDetectorNode(Node):
    def __init__(self):
        super().__init__('follower_detector_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/TB3_1/scan',
            self.scan_callback,
            rclpy.qos.qos_profile_sensor_data)
            
        self.latest_odom_tf = None
        self.timer = self.create_timer(0.05, self.publish_tf)
        self.get_logger().info("Follower detector initialized. Waiting for /TB3_1/scan...")

    def scan_callback(self, msg):
        try:
            points = []
            for i, r in enumerate(msg.ranges):
                if r < msg.range_min or r > msg.range_max or math.isinf(r) or math.isnan(r):
                    continue
                ang = msg.angle_min + i * msg.angle_increment
                # Look mostly behind TB3_1 (angles > 2.0 or < -2.0 radians)
                if abs(ang) > 2.0: 
                    if 0.15 < r < 1.0:
                        x = r * math.cos(ang)
                        y = r * math.sin(ang)
                        points.append((x, y))
            
            if not points:
                return
                
            points.sort(key=lambda p: math.hypot(p[0], p[1]))
            closest = points[0]
            cluster = [p for p in points if math.hypot(p[0]-closest[0], p[1]-closest[1]) < 0.15]
            avg_x = sum(p[0] for p in cluster) / len(cluster)
            avg_y = sum(p[1] for p in cluster) / len(cluster)
            
            dist = math.hypot(avg_x, avg_y)
            center_dist = dist + 0.10  # adjust to center of follower
            angle = math.atan2(avg_y, avg_x)
            center_x = center_dist * math.cos(angle)
            center_y = center_dist * math.sin(angle)

            pt_in_scan = PointStamped()
            pt_in_scan.header = msg.header
            pt_in_scan.point.x = center_x
            pt_in_scan.point.y = center_y
            pt_in_scan.point.z = 0.0

            transform_scan_to_odom1 = self.tf_buffer.lookup_transform(
                'TB3_1/odom',
                'TB3_1/base_scan',
                rclpy.time.Time())
            
            pt_in_odom1 = tf2_geometry_msgs.do_transform_point(pt_in_scan, transform_scan_to_odom1)

            transform_odom2_to_base2 = self.tf_buffer.lookup_transform(
                'TB3_2/odom',
                'TB3_2/base_footprint',
                rclpy.time.Time())
            
            new_tf = TransformStamped()
            new_tf.header.frame_id = 'TB3_1/odom'
            new_tf.child_frame_id = 'TB3_2/odom'
            
            new_tf.transform.translation.x = pt_in_odom1.point.x - transform_odom2_to_base2.transform.translation.x
            new_tf.transform.translation.y = pt_in_odom1.point.y - transform_odom2_to_base2.transform.translation.y
            new_tf.transform.translation.z = pt_in_odom1.point.z - transform_odom2_to_base2.transform.translation.z
            new_tf.transform.rotation.w = 1.0
            
            if self.latest_odom_tf is None:
                self.latest_odom_tf = new_tf
                self.get_logger().info(f"Initial follower detection at distance {dist:.2f}m")
            else:
                alpha = 0.05
                self.latest_odom_tf.transform.translation.x = (1-alpha)*self.latest_odom_tf.transform.translation.x + alpha*new_tf.transform.translation.x
                self.latest_odom_tf.transform.translation.y = (1-alpha)*self.latest_odom_tf.transform.translation.y + alpha*new_tf.transform.translation.y
                
        except Exception as e:
            pass

    def publish_tf(self):
        now = self.get_clock().now().to_msg()
        if self.latest_odom_tf is not None:
            self.latest_odom_tf.header.stamp = now
            self.tf_broadcaster.sendTransform(self.latest_odom_tf)
        else:
            new_tf = TransformStamped()
            new_tf.header.frame_id = 'TB3_1/odom'
            new_tf.header.stamp = now
            new_tf.child_frame_id = 'TB3_2/odom'
            new_tf.transform.translation.x = -0.4
            new_tf.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(new_tf)

def main(args=None):
    rclpy.init(args=args)
    node = FollowerDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

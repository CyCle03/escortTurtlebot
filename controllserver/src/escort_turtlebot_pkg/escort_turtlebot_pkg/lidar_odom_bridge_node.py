#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def _pose_to_mat(x: float, y: float, yaw: float):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return [
        [c, -s, x],
        [s, c, y],
        [0.0, 0.0, 1.0],
    ]


def _mat_mul(a, b):
    return [
        [
            a[0][0] * b[0][0] + a[0][1] * b[1][0],
            a[0][0] * b[0][1] + a[0][1] * b[1][1],
            a[0][0] * b[0][2] + a[0][1] * b[1][2] + a[0][2],
        ],
        [
            a[1][0] * b[0][0] + a[1][1] * b[1][0],
            a[1][0] * b[0][1] + a[1][1] * b[1][1],
            a[1][0] * b[0][2] + a[1][1] * b[1][2] + a[1][2],
        ],
        [0.0, 0.0, 1.0],
    ]


def _mat_inv(m):
    r00 = m[0][0]
    r01 = m[0][1]
    r10 = m[1][0]
    r11 = m[1][1]
    tx = m[0][2]
    ty = m[1][2]
    return [
        [r00, r10, -(r00 * tx + r10 * ty)],
        [r01, r11, -(r01 * tx + r11 * ty)],
        [0.0, 0.0, 1.0],
    ]


def _mat_to_pose(m):
    x = m[0][2]
    y = m[1][2]
    yaw = math.atan2(m[1][0], m[0][0])
    return x, y, yaw


class LidarOdomBridgeNode(Node):
    def __init__(self):
        super().__init__('lidar_odom_bridge')
        self.declare_parameter('scan_topic', '/TB3_2/scan')
        self.declare_parameter('leader_ns', 'TB3_1')
        self.declare_parameter('follower_ns', 'TB3_2')
        self.declare_parameter('search_half_angle_deg', 25.0)
        self.declare_parameter('target_bearing_deg', 0.0)
        self.declare_parameter('min_range', 0.12)
        self.declare_parameter('max_range', 3.0)
        self.declare_parameter('smoothing_alpha', 0.35)
        self.declare_parameter('use_sim_time', False)

        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.leader_ns = self.get_parameter('leader_ns').get_parameter_value().string_value
        self.follower_ns = self.get_parameter('follower_ns').get_parameter_value().string_value
        self.half_angle = math.radians(
            self.get_parameter('search_half_angle_deg').get_parameter_value().double_value
        )
        self.target_bearing = math.radians(
            self.get_parameter('target_bearing_deg').get_parameter_value().double_value
        )
        self.min_range = self.get_parameter('min_range').get_parameter_value().double_value
        self.max_range = self.get_parameter('max_range').get_parameter_value().double_value
        self.alpha = self.get_parameter('smoothing_alpha').get_parameter_value().double_value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub = self.create_subscription(LaserScan, scan_topic, self._scan_cb, qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_fb_lb: Optional[Tuple[float, float]] = None

    def _estimate_leader_in_scan(self, msg: LaserScan) -> Optional[Tuple[float, float]]:
        candidates = []
        for i, rng in enumerate(msg.ranges):
            if not math.isfinite(rng) or rng < self.min_range or rng > self.max_range:
                continue
            ang = msg.angle_min + i * msg.angle_increment
            d = math.atan2(math.sin(ang - self.target_bearing), math.cos(ang - self.target_bearing))
            if abs(d) <= self.half_angle:
                candidates.append((rng, ang))
        if not candidates:
            return None

        candidates.sort(key=lambda x: x[0])
        top = candidates[: min(7, len(candidates))]
        r = sum(v[0] for v in top) / len(top)
        a = sum(v[1] for v in top) / len(top)
        x = r * math.cos(a)
        y = r * math.sin(a)
        if self.last_fb_lb is None:
            self.last_fb_lb = (x, y)
        else:
            self.last_fb_lb = (
                self.alpha * x + (1.0 - self.alpha) * self.last_fb_lb[0],
                self.alpha * y + (1.0 - self.alpha) * self.last_fb_lb[1],
            )
        return self.last_fb_lb

    def _lookup_2d(self, parent: str, child: str) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(parent, child, Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            return t.x, t.y, _quat_to_yaw(q.x, q.y, q.z, q.w)
        except TransformException:
            return None

    def _scan_cb(self, msg: LaserScan):
        fb_lb = self._estimate_leader_in_scan(msg)
        if fb_lb is None:
            return

        leader_odom = f'{self.leader_ns}/odom'
        follower_odom = f'{self.follower_ns}/odom'
        leader_base = f'{self.leader_ns}/base_footprint'
        follower_base = f'{self.follower_ns}/base_footprint'

        lo_lb = self._lookup_2d(leader_odom, leader_base)
        fo_fb = self._lookup_2d(follower_odom, follower_base)
        if lo_lb is None or fo_fb is None:
            return

        t_lo_lb = _pose_to_mat(*lo_lb)
        # LiDAR gives follower_base -> leader_base translation; yaw is approximated as 0.
        t_fb_lb = _pose_to_mat(fb_lb[0], fb_lb[1], 0.0)
        t_lb_fb = _mat_inv(t_fb_lb)
        t_fo_fb = _pose_to_mat(*fo_fb)
        t_fb_fo = _mat_inv(t_fo_fb)
        t_lo_fo = _mat_mul(_mat_mul(t_lo_lb, t_lb_fb), t_fb_fo)
        x, y, yaw = _mat_to_pose(t_lo_fo)
        qx, qy, qz, qw = _yaw_to_quat(yaw)

        out = TransformStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = leader_odom
        out.child_frame_id = follower_odom
        out.transform.translation.x = float(x)
        out.transform.translation.y = float(y)
        out.transform.translation.z = 0.0
        out.transform.rotation.x = qx
        out.transform.rotation.y = qy
        out.transform.rotation.z = qz
        out.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(out)


def main():
    rclpy.init()
    node = LidarOdomBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

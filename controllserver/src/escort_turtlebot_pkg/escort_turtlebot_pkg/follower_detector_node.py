#!/usr/bin/env python3

import math

from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial import cKDTree
from sensor_msgs.msg import LaserScan
import tf2_ros


def get_transform_matrix_2d(transform_msg):
    t = transform_msg.transform.translation
    q = transform_msg.transform.rotation
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    theta = math.atan2(siny_cosp, cosy_cosp)

    T = np.identity(3)
    T[0, 0] = math.cos(theta)
    T[0, 1] = -math.sin(theta)
    T[1, 0] = math.sin(theta)
    T[1, 1] = math.cos(theta)
    T[0, 2] = t.x
    T[1, 2] = t.y
    return T


def matrix_to_transform_msg_2d(T, frame_id, child_frame_id, stamp):
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.child_frame_id = child_frame_id

    msg.transform.translation.x = float(T[0, 2])
    msg.transform.translation.y = float(T[1, 2])
    msg.transform.translation.z = 0.0

    theta = math.atan2(T[1, 0], T[0, 0])
    msg.transform.rotation.x = 0.0
    msg.transform.rotation.y = 0.0
    msg.transform.rotation.z = math.sin(theta / 2.0)
    msg.transform.rotation.w = math.cos(theta / 2.0)
    return msg


def scan_to_points(msg, max_range=4.0):
    points = []
    for i, r in enumerate(msg.ranges):
        if r < msg.range_min or r > min(msg.range_max, max_range) or math.isinf(r) or math.isnan(r):
            continue
        ang = msg.angle_min + i * msg.angle_increment
        x = r * math.cos(ang)
        y = r * math.sin(ang)
        points.append([x, y])
    if len(points) == 0:
        return np.array([])
    pts = np.array(points)
    step = max(1, len(pts) // 150)
    return pts[::step, :]


def icp(A, B, init_pose, max_iterations=30, tolerance=0.0001):
    src = np.ones((3, A.shape[0]))
    src[:2, :] = A.T
    dst = np.ones((3, B.shape[0]))
    dst[:2, :] = B.T

    tx, ty, theta = init_pose
    T = np.array([
        [np.cos(theta), -np.sin(theta), tx],
        [np.sin(theta), np.cos(theta), ty],
        [0, 0, 1]
    ])

    src = np.dot(T, src)
    prev_error = float('inf')

    tree = cKDTree(dst[:2, :].T)

    for i in range(max_iterations):
        distances, indices = tree.query(src[:2, :].T)
        valid = distances < 0.4
        if np.sum(valid) < 10:
            break

        src_valid = src[:2, valid].T
        dst_valid = dst[:2, indices[valid]].T

        centroid_A = np.mean(src_valid, axis=0)
        centroid_B = np.mean(dst_valid, axis=0)

        AA = src_valid - centroid_A
        BB = dst_valid - centroid_B

        H = np.dot(AA.T, BB)
        U, S, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)

        if np.linalg.det(R) < 0:
            Vt[1, :] *= -1
            R = np.dot(Vt.T, U.T)

        t = centroid_B.T - np.dot(R, centroid_A.T)

        T_update = np.identity(3)
        T_update[:2, :2] = R
        T_update[:2, 2] = t

        src = np.dot(T_update, src)
        T = np.dot(T_update, T)

        mean_error = np.mean(distances[valid])
        if abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error

    final_tx = T[0, 2]
    final_ty = T[1, 2]
    final_theta = math.atan2(T[1, 0], T[0, 0])

    distances, _ = tree.query(src[:2, :].T)
    fitness = np.mean(distances < 0.15)

    return np.array([final_tx, final_ty, final_theta]), fitness


class FollowerDetectorNode(Node):

    def __init__(self):
        super().__init__('follower_detector_node')
        try:
            self.declare_parameter('use_sim_time', False)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass
        self.declare_parameter('leader_name', 'TB3_1')
        self.declare_parameter('follower_name', 'TB3_2')

        # ICP 매칭 품질 임계값: 이 값 이상이어야 TF 업데이트에 사용
        self.declare_parameter('icp_fitness_threshold', 0.3)
        # 이전 TF와 새 ICP 결과를 혼합하는 비율 (0=이전값 유지, 1=ICP 결과 그대로 사용)
        self.declare_parameter('blend_alpha', 0.5)
        # 스캔이 이 시간(초) 이상 수신되지 않으면 stale 스캔으로 간주하여
        # 마지막 유효한 bridge TF를 보존하기 위해 ICP를 건너 뜀
        self.declare_parameter('scan_timeout_sec', 1.0)
        # ICP 결과가 현재 bridge TF 대비 이 거리(단위: m) 이상 차이가 나면 부정확한 결과로 판단 → reject
        self.declare_parameter('max_correction_dist', 0.3)
        # ICP 결과가 현재 bridge TF 대비 이 각도(rad) 이상 차이가 나면 reject
        self.declare_parameter('max_correction_angle', 0.5)
        # 팔로워가 이동 중인지 판단하는 init_pose 변화량 임계값(m).
        # 이 값 이하면 팔로워가 정지한 것으로 간주하고 ICP 업데이트를 건너뜀 (맵 오염 원천 차단)
        self.declare_parameter('odom_motion_threshold', 0.02)

        self.leader_name = self.get_parameter('leader_name').value
        self.follower_name = self.get_parameter('follower_name').value
        self.icp_fitness_threshold = self.get_parameter('icp_fitness_threshold').value
        self.blend_alpha = self.get_parameter('blend_alpha').value
        self.scan_timeout_sec = self.get_parameter('scan_timeout_sec').value
        self.max_correction_dist = self.get_parameter('max_correction_dist').value
        self.max_correction_angle = self.get_parameter('max_correction_angle').value
        self.odom_motion_threshold = self.get_parameter('odom_motion_threshold').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.subscription1 = self.create_subscription(
            LaserScan,
            f'/{self.leader_name}/scan',
            self.scan1_callback,
            rclpy.qos.qos_profile_sensor_data)

        self.subscription2 = self.create_subscription(
            LaserScan,
            f'/{self.follower_name}/scan',
            self.scan2_callback,
            rclpy.qos.qos_profile_sensor_data)

        self.latest_odom_tf = None
        self.latest_scan1 = None
        self.latest_scan1_stamp = None  # 리더 스캔 마지막 수신 시각 (stale 감지용)
        self.prev_init_pose = None      # 팔로워 정지 상태 감지용: 직전 ICP init_pose 저장
        self.timer = self.create_timer(0.05, self.publish_tf)
        self.get_logger().info(
            f"ICP Scan Matching Follower detector for {self.follower_name} "
            f"following {self.leader_name} initialized! "
            f'(fitness={self.icp_fitness_threshold:.2f}, '
            f'alpha={self.blend_alpha:.2f}, '
            f'scan_timeout={self.scan_timeout_sec:.1f}s, '
            f'max_corr={self.max_correction_dist:.2f}m/'
            f'{math.degrees(self.max_correction_angle):.0f}deg, '
            f'odom_thresh={self.odom_motion_threshold:.3f}m)')

    def scan1_callback(self, msg):
        pts = scan_to_points(msg, max_range=3.5)
        if len(pts) > 10:
            self.latest_scan1 = pts
            self.latest_scan1_stamp = self.get_clock().now()  # 수신 시각 기록

    def scan2_callback(self, msg):
        if self.latest_scan1 is None:
            return

        # 리더 스캔이 오래된 데이터인지 확인 (리더봇 소실 감지)
        if self.latest_scan1_stamp is not None:
            scan1_age = (self.get_clock().now() - self.latest_scan1_stamp).nanoseconds / 1e9
            if scan1_age > self.scan_timeout_sec:
                # stale 스캔으로 ICP를 실행하면 bridge TF가 오염될 수 있음
                # 마지막 유효한 TF를 publish_tf()로 유지하고 건너뜀
                self.get_logger().warn(
                    f'{self.leader_name}/scan not received for {scan1_age:.1f}s '
                    '(리더봇 소실 추정). 마지막 bridge TF를 보존합니다.',
                    throttle_duration_sec=2.0)
                return

        pts2 = scan_to_points(msg, max_range=3.5)
        if len(pts2) < 10:
            return

        try:
            T_odom1_scan1_msg = self.tf_buffer.lookup_transform(
                f'{self.leader_name}/odom', f'{self.leader_name}/base_scan',
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
            T_odom2_scan2_msg = self.tf_buffer.lookup_transform(
                f'{self.follower_name}/odom', f'{self.follower_name}/base_scan',
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))

            T_odom1_scan1 = get_transform_matrix_2d(T_odom1_scan1_msg)
            T_odom2_scan2 = get_transform_matrix_2d(T_odom2_scan2_msg)

            if self.latest_odom_tf is not None:
                T_odom1_odom2 = get_transform_matrix_2d(self.latest_odom_tf)
                T_scan1_odom1 = np.linalg.inv(T_odom1_scan1)
                T_scan1_scan2_guess = T_scan1_odom1 @ T_odom1_odom2 @ T_odom2_scan2
                init_pose = [
                    T_scan1_scan2_guess[0, 2],
                    T_scan1_scan2_guess[1, 2],
                    math.atan2(T_scan1_scan2_guess[1, 0], T_scan1_scan2_guess[0, 0])
                ]
            else:
                # 초기 추정: 팔로워가 리더의 약 0.5m 뒤에 있음
                init_pose = [-0.5, 0.0, 0.0]

            # --- 팔로워 정지 상태 감지: map 오염 원천 차단 ---
            # init_pose는 팔로워의 odom 변화를 반영하므로,
            # 직전 init_pose와 현재 init_pose를 비교하여 팔로워가 멈췄는지 판단
            if self.prev_init_pose is not None:
                pose_change = math.hypot(
                    init_pose[0] - self.prev_init_pose[0],
                    init_pose[1] - self.prev_init_pose[1])
                if pose_change < self.odom_motion_threshold:
                    # 팔로워가 정지.
                    # 환경이 다른 두 스캔을 매칭하면 bridge TF가 오염되어
                    # map이 깨질 수 있으므로 ICP를 건너뜀.
                    self.get_logger().info(
                        f'{self.follower_name} stationary '
                        f'(odom_delta={pose_change:.4f}m < {self.odom_motion_threshold:.4f}m). '
                        'Skipping ICP to protect bridge TF.',
                        throttle_duration_sec=3.0)
                    self.prev_init_pose = init_pose
                    return
            self.prev_init_pose = init_pose

            T_icp_pose, fitness = icp(pts2, self.latest_scan1, init_pose)

            if fitness < self.icp_fitness_threshold:
                # ICP 매칭 점수가 너무 낮으면 무시 (신뢰할 수 없는 결과)
                self.get_logger().warn(
                    f'ICP fitness too low ({fitness:.2f} < {self.icp_fitness_threshold:.2f}), '
                    'skipping update.',
                    throttle_duration_sec=2.0)
                return

            tx, ty, theta = T_icp_pose
            T_scan1_scan2 = np.array([
                [math.cos(theta), -math.sin(theta), tx],
                [math.sin(theta), math.cos(theta), ty],
                [0, 0, 1]
            ])

            T_scan2_odom2 = np.linalg.inv(T_odom2_scan2)
            T_odom1_odom2_new = T_odom1_scan1 @ T_scan1_scan2 @ T_scan2_odom2

            # --- ICP 보정 크기 필터: 큰 TF 점프 reject ---
            if self.latest_odom_tf is not None:
                T_curr_bridge = get_transform_matrix_2d(self.latest_odom_tf)
                corr_dist = math.hypot(
                    T_odom1_odom2_new[0, 2] - T_curr_bridge[0, 2],
                    T_odom1_odom2_new[1, 2] - T_curr_bridge[1, 2])
                th_curr_b = math.atan2(T_curr_bridge[1, 0], T_curr_bridge[0, 0])
                th_new_b = math.atan2(T_odom1_odom2_new[1, 0], T_odom1_odom2_new[0, 0])
                corr_angle = abs(math.atan2(math.sin(th_new_b - th_curr_b),
                                            math.cos(th_new_b - th_curr_b)))
                if corr_dist > self.max_correction_dist or corr_angle > self.max_correction_angle:
                    self.get_logger().warn(
                        'ICP correction too large '
                        f'(dist={corr_dist:.2f}m/{self.max_correction_dist:.2f}m, '
                        f'angle={math.degrees(corr_angle):.1f}/'
                        f'{math.degrees(self.max_correction_angle):.1f}deg). '
                        'Rejecting to protect bridge TF.',
                        throttle_duration_sec=2.0)
                    return

            now = self.get_clock().now().to_msg()
            new_msg = matrix_to_transform_msg_2d(
                T_odom1_odom2_new, f'{self.leader_name}/odom', f'{self.follower_name}/odom', now)

            if self.latest_odom_tf is None:
                self.latest_odom_tf = new_msg
                self.get_logger().info(
                    f'ICP Initialized! Fitness: {fitness:.2f}, '
                    f'distance: {math.hypot(tx, ty):.2f}m')
            else:
                alpha = self.blend_alpha
                T_curr = get_transform_matrix_2d(self.latest_odom_tf)
                T_new = get_transform_matrix_2d(new_msg)

                tx_blend = (1 - alpha) * T_curr[0, 2] + alpha * T_new[0, 2]
                ty_blend = (1 - alpha) * T_curr[1, 2] + alpha * T_new[1, 2]

                th_curr = math.atan2(T_curr[1, 0], T_curr[0, 0])
                th_new = math.atan2(T_new[1, 0], T_new[0, 0])

                diff = th_new - th_curr
                while diff > math.pi:
                    diff -= 2 * math.pi
                while diff < -math.pi:
                    diff += 2 * math.pi
                th_blend = th_curr + alpha * diff

                T_blend = np.array([
                    [math.cos(th_blend), -math.sin(th_blend), tx_blend],
                    [math.sin(th_blend), math.cos(th_blend), ty_blend],
                    [0, 0, 1]
                ])

                self.latest_odom_tf = matrix_to_transform_msg_2d(
                    T_blend, f'{self.leader_name}/odom', f'{self.follower_name}/odom', now)

        except Exception as e:
            self.get_logger().warn(
                f'scan2_callback error: {e}',
                throttle_duration_sec=2.0)

    def publish_tf(self):
        now = self.get_clock().now().to_msg()
        if self.latest_odom_tf is not None:
            self.latest_odom_tf.header.stamp = now
            self.tf_broadcaster.sendTransform(self.latest_odom_tf)
        else:
            new_tf = TransformStamped()
            new_tf.header.frame_id = f'{self.leader_name}/odom'
            new_tf.header.stamp = now
            new_tf.child_frame_id = f'{self.follower_name}/odom'
            new_tf.transform.translation.x = -0.5
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

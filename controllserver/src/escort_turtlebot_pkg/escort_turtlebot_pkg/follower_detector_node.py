#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf2_geometry_msgs
import math
import numpy as np
from scipy.spatial import cKDTree

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
        [np.sin(theta),  np.cos(theta), ty],
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
            Vt[1,:] *= -1
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
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.subscription1 = self.create_subscription(
            LaserScan,
            '/TB3_1/scan',
            self.scan1_callback,
            rclpy.qos.qos_profile_sensor_data)
            
        self.subscription2 = self.create_subscription(
            LaserScan,
            '/TB3_2/scan',
            self.scan2_callback,
            rclpy.qos.qos_profile_sensor_data)
            
        self.latest_odom_tf = None
        self.latest_scan1 = None
        self.timer = self.create_timer(0.05, self.publish_tf)
        self.get_logger().info("ICP Scan Matching Follower detector initialized!")

    def scan1_callback(self, msg):
        pts = scan_to_points(msg, max_range=3.5)
        if len(pts) > 10:
            self.latest_scan1 = pts

    def scan2_callback(self, msg):
        if self.latest_scan1 is None:
            return
            
        pts2 = scan_to_points(msg, max_range=3.5)
        if len(pts2) < 10:
            return
            
        try:
            T_odom1_scan1_msg = self.tf_buffer.lookup_transform(
                'TB3_1/odom', 'TB3_1/base_scan', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
            T_odom2_scan2_msg = self.tf_buffer.lookup_transform(
                'TB3_2/odom', 'TB3_2/base_scan', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
            
            T_odom1_scan1 = get_transform_matrix_2d(T_odom1_scan1_msg)
            T_odom2_scan2 = get_transform_matrix_2d(T_odom2_scan2_msg)
            
            if self.latest_odom_tf is not None:
                T_odom1_odom2 = get_transform_matrix_2d(self.latest_odom_tf)
                T_scan1_odom1 = np.linalg.inv(T_odom1_scan1)
                T_scan1_scan2_guess = T_scan1_odom1 @ T_odom1_odom2 @ T_odom2_scan2
                init_pose = [T_scan1_scan2_guess[0, 2], T_scan1_scan2_guess[1, 2], math.atan2(T_scan1_scan2_guess[1, 0], T_scan1_scan2_guess[0, 0])]
            else:
                # 초기 추정: TB3_2가 TB3_1의 약 0.7m 뒤에 있음
                init_pose = [-0.7, 0.0, 0.0]
                
            T_icp_pose, fitness = icp(pts2, self.latest_scan1, init_pose)
            
            if fitness < 0.15:
                # 매칭 실패 시 너무 낮은 점수면 무시
                return
                
            tx, ty, theta = T_icp_pose
            T_scan1_scan2 = np.array([
                [math.cos(theta), -math.sin(theta), tx],
                [math.sin(theta),  math.cos(theta), ty],
                [0, 0, 1]
            ])
            
            T_scan2_odom2 = np.linalg.inv(T_odom2_scan2)
            T_odom1_odom2_new = T_odom1_scan1 @ T_scan1_scan2 @ T_scan2_odom2
            
            now = self.get_clock().now().to_msg()
            new_msg = matrix_to_transform_msg_2d(T_odom1_odom2_new, 'TB3_1/odom', 'TB3_2/odom', now)
            
            if self.latest_odom_tf is None:
                self.latest_odom_tf = new_msg
                self.get_logger().info(f"ICP Initialized! Fitness: {fitness:.2f}, distance: {math.hypot(tx, ty):.2f}m")
            else:
                alpha = 0.5
                T_curr = get_transform_matrix_2d(self.latest_odom_tf)
                T_new = get_transform_matrix_2d(new_msg)
                
                tx_blend = (1-alpha)*T_curr[0,2] + alpha*T_new[0,2]
                ty_blend = (1-alpha)*T_curr[1,2] + alpha*T_new[1,2]
                
                th_curr = math.atan2(T_curr[1,0], T_curr[0,0])
                th_new = math.atan2(T_new[1,0], T_new[0,0])
                
                diff = th_new - th_curr
                while diff > math.pi: diff -= 2*math.pi
                while diff < -math.pi: diff += 2*math.pi
                th_blend = th_curr + alpha * diff
                
                T_blend = np.array([
                    [math.cos(th_blend), -math.sin(th_blend), tx_blend],
                    [math.sin(th_blend),  math.cos(th_blend), ty_blend],
                    [0, 0, 1]
                ])
                
                self.latest_odom_tf = matrix_to_transform_msg_2d(T_blend, 'TB3_1/odom', 'TB3_2/odom', now)
                
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
            new_tf.transform.translation.x = -0.7
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

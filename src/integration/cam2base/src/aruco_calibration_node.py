#!/usr/bin/env python3
"""
ArUco Calibration Node

Uses RealSense camera to detect ArUco marker and compute transform:
camera → aruco (via OpenCV detection)
camera → robot_base (from hand-eye calibration TF)
---
Result: aruco → robot_base = inverse(camera → aruco) * (camera → robot_base)

This replaces manual measurement with automatic camera-based calibration.

Usage:
    ros2 run cam2base aruco_calibration_node

The node will:
1. Subscribe to RealSense RGB image
2. Detect ArUco marker (DICT_6X6_250, ID=0)
3. Compute camera → aruco transform using cv2.aruco
4. Lookup camera → lbr_link_0 from TF tree
5. Compute aruco → lbr_link_0 transform
6. Print the transform to copy into aruco_tf_publisher.cpp
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import tf2_ros
from tf2_ros import TransformException
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
import sys


class ArucoCalibrationNode(Node):
    def __init__(self):
        super().__init__('aruco_calibration_node')
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('ArUco Calibration Node Started')
        self.get_logger().info('=' * 60)
        
        # ArUco detection parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.marker_id = 0
        self.marker_size = 0.15  # 15cm = 0.15 meters
        
        # Camera intrinsics (RealSense D435 default, update if needed)
        # These should match your actual camera calibration
        self.camera_matrix = np.array([
            [615.0, 0.0, 320.0],
            [0.0, 615.0, 240.0],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.zeros(5)  # Assume no distortion for RealSense
        
        # TF2 for getting camera → robot_base
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Subscribe to RealSense color image
        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_rect_raw',
            self.image_callback,
            10
        )
        
        # Calibration state
        self.calibration_complete = False
        self.detection_count = 0
        self.required_detections = 1  # Just need 1 detection for quick calibration
        self.accumulated_transforms = []
        
        self.get_logger().info('Waiting for ArUco marker detection...')
        self.get_logger().info('Make sure ArUco marker (ID=0, 15cm) is visible to camera')
    
    def image_callback(self, msg: Image):
        if self.calibration_complete:
            return
        
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Detect ArUco markers
            corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
            
            if ids is not None and self.marker_id in ids:
                # Find the index of our target marker
                idx = np.where(ids == self.marker_id)[0][0]
                marker_corners = corners[idx]
                
                # Estimate pose of ArUco marker
                # For OpenCV 4.7+, use solvePnP instead of deprecated estimatePoseSingleMarkers
                obj_points = np.array([
                    [-self.marker_size/2, self.marker_size/2, 0],
                    [self.marker_size/2, self.marker_size/2, 0],
                    [self.marker_size/2, -self.marker_size/2, 0],
                    [-self.marker_size/2, -self.marker_size/2, 0]
                ], dtype=np.float32)
                
                success, rvec, tvec = cv2.solvePnP(
                    obj_points,
                    marker_corners.reshape(-1, 2),
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
                
                # Convert to transform matrix
                cam_to_aruco = self.pose_to_transform(rvec, tvec)
                
                # Accumulate detections
                self.accumulated_transforms.append(cam_to_aruco)
                self.detection_count += 1
                
                # Draw detection on image for visual feedback
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs,
                                rvec, tvec, 0.1)
                
                self.get_logger().info(f'ArUco marker detected! Computing calibration...')
                
                # Show image with detection (optional, comment out if headless)
                cv2.imshow('ArUco Detection', cv_image)
                cv2.waitKey(1)
                
                # Once we have enough detections, compute final transform
                if self.detection_count >= self.required_detections:
                    self.compute_final_calibration()
                    self.calibration_complete = True
            
            else:
                # No marker detected, show image anyway
                cv2.imshow('ArUco Detection (waiting...)', cv_image)
                cv2.waitKey(1)
        
        except Exception as e:
            self.get_logger().error(f'Error in image callback: {e}')
    
    def pose_to_transform(self, rvec, tvec):
        """Convert rotation vector and translation vector to 4x4 transform matrix"""
        rot_matrix, _ = cv2.Rodrigues(rvec)
        transform = np.eye(4)
        transform[:3, :3] = rot_matrix
        transform[:3, 3] = tvec.flatten()
        return transform
    
    def compute_final_calibration(self):
        """Average accumulated detections and compute aruco → robot_base transform"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('Computing final calibration...')
        self.get_logger().info('=' * 60)
        
        # Average the accumulated transforms (simple mean of translations and rotations)
        avg_translation = np.mean([t[:3, 3] for t in self.accumulated_transforms], axis=0)
        avg_rotation = self.average_rotations([t[:3, :3] for t in self.accumulated_transforms])
        
        cam_to_aruco = np.eye(4)
        cam_to_aruco[:3, :3] = avg_rotation
        cam_to_aruco[:3, 3] = avg_translation
        
        self.get_logger().info('Averaged camera → aruco transform computed')
        
        # Get camera → robot_base from TF tree
        # Wait a bit for TF tree to be fully populated
        import time
        time.sleep(1.0)
        
        try:
            # Use Time(0) to get the latest available transform
            transform_stamped = self.tf_buffer.lookup_transform(
                'lbr_link_0',      # target frame (robot base)
                'camera_frame',    # source frame
                rclpy.time.Time(seconds=0),  # Get latest available
                timeout=rclpy.duration.Duration(seconds=10.0)
            )
            
            cam_to_base = self.transform_stamped_to_matrix(transform_stamped)
            self.get_logger().info('Retrieved camera → lbr_link_0 from TF tree')
            
        except TransformException as e:
            self.get_logger().error(f'Failed to get camera → robot_base transform: {e}')
            self.get_logger().error('Make sure hand2eye_tf_publisher and robot state publisher are running!')
            self.get_logger().error('Available frames in TF tree:')
            self.get_logger().error(str(self.tf_buffer.all_frames_as_string()))
            return
        
        # Compute aruco → robot_base
        # aruco_to_base = base_to_camera * camera_to_aruco
        # BUT we have camera_to_base, so:
        # aruco_to_base = inverse(aruco_to_camera) * camera_to_base
        aruco_to_cam = np.linalg.inv(cam_to_aruco)
        aruco_to_base = aruco_to_cam @ cam_to_base
        
        # Also compute base → aruco (what we actually need for TF publisher)
        base_to_aruco = np.linalg.inv(aruco_to_base)
        
        self.print_calibration_results(base_to_aruco)
    
    def average_rotations(self, rotation_matrices):
        """Average multiple rotation matrices using quaternion averaging"""
        quaternions = [R.from_matrix(rot).as_quat() for rot in rotation_matrices]
        avg_quat = np.mean(quaternions, axis=0)
        avg_quat = avg_quat / np.linalg.norm(avg_quat)  # Normalize
        return R.from_quat(avg_quat).as_matrix()
    
    def transform_stamped_to_matrix(self, transform: TransformStamped):
        """Convert TransformStamped to 4x4 matrix"""
        t = transform.transform.translation
        q = transform.transform.rotation
        
        translation = np.array([t.x, t.y, t.z])
        rotation = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        
        matrix = np.eye(4)
        matrix[:3, :3] = rotation
        matrix[:3, 3] = translation
        return matrix
    
    def print_calibration_results(self, base_to_aruco):
        """Print results in format ready to copy into aruco_tf_publisher.cpp"""
        translation = base_to_aruco[:3, 3]
        rotation = R.from_matrix(base_to_aruco[:3, :3])
        quat = rotation.as_quat()  # [x, y, z, w]
        euler = rotation.as_euler('xyz', degrees=True)
        
        print('\n')
        print('=' * 70)
        print('ARUCO CALIBRATION COMPLETE!')
        print('=' * 70)
        print('\nTransform: lbr_link_0 → aruco_marker')
        print('\n--- TRANSLATION (meters) ---')
        print(f'  X: {translation[0]:+.6f}')
        print(f'  Y: {translation[1]:+.6f}')
        print(f'  Z: {translation[2]:+.6f}')
        print('\n--- ROTATION (quaternion) ---')
        print(f'  x: {quat[0]:+.6f}')
        print(f'  y: {quat[1]:+.6f}')
        print(f'  z: {quat[2]:+.6f}')
        print(f'  w: {quat[3]:+.6f}')
        print('\n--- ROTATION (Euler angles, degrees) ---')
        print(f'  Roll  (X): {euler[0]:+.2f}°')
        print(f'  Pitch (Y): {euler[1]:+.2f}°')
        print(f'  Yaw   (Z): {euler[2]:+.2f}°')
        print('\n' + '=' * 70)
        print('COPY THIS INTO aruco_tf_publisher.cpp:')
        print('=' * 70)
        print('\n// Translation')
        print(f'transform.transform.translation.x = {translation[0]:.6f};')
        print(f'transform.transform.translation.y = {translation[1]:.6f};')
        print(f'transform.transform.translation.z = {translation[2]:.6f};')
        print('\n// Rotation (quaternion)')
        print(f'tf2::Quaternion q;')
        print(f'q.setX({quat[0]:.6f});')
        print(f'q.setY({quat[1]:.6f});')
        print(f'q.setZ({quat[2]:.6f});')
        print(f'q.setW({quat[3]:.6f});')
        print('\ntransform.transform.rotation.x = q.x();')
        print('transform.transform.rotation.y = q.y();')
        print('transform.transform.rotation.z = q.z();')
        print('transform.transform.rotation.w = q.w();')
        print('\n' + '=' * 70)
        print('\nCalibration complete! Update aruco_tf_publisher.cpp and rebuild.')
        print('=' * 70)
        
        self.get_logger().info('Calibration complete! See console output above.')
        self.get_logger().info('Node will now exit. Press Ctrl+C or close window.')


def main(args=None):
    rclpy.init(args=args)
    node = ArucoCalibrationNode()
    
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


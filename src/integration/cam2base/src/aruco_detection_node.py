#!/usr/bin/env python3
"""
ArUco Detection Node

Detects ArUco marker and computes transform from lbr_link_0 to aruco_marker.
Detects for a few seconds, averages the results, then freezes and prints the transform.

Usage:
    ros2 run cam2base aruco_detection_node
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import tf2_ros
from tf2_ros import TransformException
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped, PoseStamped
from scipy.spatial.transform import Rotation as R
import time


class ArucoDetectionNode(Node):
    def __init__(self):
        super().__init__('aruco_detection_node')
        
        self.get_logger().info('ArUco Detection Node Started')
        
        # ArUco detection parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.marker_id = 0
        self.marker_size = 0.17  # 17cm = 0.17 meters
        
        # Camera intrinsics (PLACEHOLDER - user will update)
        # TODO: Replace with actual camera calibration values
        fx = 425.19189453125
        fy = 424.6562805175781
        cx = 422.978515625
        cy = 242.1155242919922
        self.camera_matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
        self.dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        
        # TF2 for getting camera → robot_base
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Subscribe to camera image
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_rect_raw',
            self.image_callback,
            10
        )
        
        # Publisher for detected pose (visualizable in RVIZ)
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/aruco_detected_pose_new',
            10
        )
        
        # Detection state
        self.detection_start_time = None
        self.detection_duration = 5.0  # Detect for 5 seconds
        self.detection_complete = False
        self.accumulated_transforms = []
        self.final_pose = None
        
        # Visualization window
        self.show_window = True
        cv2.namedWindow('ArUco Detection', cv2.WINDOW_NORMAL)
        
        self.get_logger().info(f'Detecting ArUco marker (ID={self.marker_id}, size={self.marker_size}m) for {self.detection_duration}s...')
    
    def image_callback(self, msg: Image):
        if self.detection_complete:
            # After detection is complete, publish the frozen pose
            if self.final_pose is not None:
                self.final_pose.header.stamp = self.get_clock().now().to_msg()
                self.pose_publisher.publish(self.final_pose)
            return
        
        try:
            # Initialize detection start time on first callback
            if self.detection_start_time is None:
                self.detection_start_time = time.time()
            
            # Check if detection period is over
            elapsed_time = time.time() - self.detection_start_time
            if elapsed_time >= self.detection_duration:
                self.compute_final_transform()
                return
            
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Detect ArUco markers
            corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
            
            # Prepare display image
            display_image = cv_image.copy()
            elapsed_time = time.time() - self.detection_start_time if self.detection_start_time else 0
            remaining_time = max(0, self.detection_duration - elapsed_time)
            
            if ids is not None and self.marker_id in ids:
                # Find the index of our target marker
                idx = np.where(ids == self.marker_id)[0][0]
                marker_corners = corners[idx]
                
                # Estimate pose of ArUco marker in camera frame
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
                
                if success:
                    # Convert to transform matrix (camera → aruco)
                    cam_to_aruco = self.pose_to_transform(rvec, tvec)
                    
                    # Get camera → lbr_link_0 transform from TF tree
                    try:
                        transform_stamped = self.tf_buffer.lookup_transform(
                            'lbr_link_0',
                            'camera_frame',
                            rclpy.time.Time(),
                            timeout=rclpy.duration.Duration(seconds=1.0)
                        )
                        
                        # TF lookup gives us transform FROM camera_frame TO lbr_link_0
                        # This is base_to_cam (transforms point in camera frame to base frame)
                        base_to_cam = self.transform_stamped_to_matrix(transform_stamped)
                        
                        # We have:
                        # - cam_to_aruco: camera → ArUco (from OpenCV solvePnP)
                        # - base_to_cam: camera → base (from TF)
                        # We want: base_to_aruco: base → ArUco
                        # Formula: base_to_aruco = base_to_cam @ cam_to_aruco
                        base_to_aruco = base_to_cam @ cam_to_aruco
                        
                        # Accumulate for averaging
                        self.accumulated_transforms.append(base_to_aruco)
                        
                        # Visual feedback - draw detected marker and axes
                        cv2.aruco.drawDetectedMarkers(display_image, corners, ids)
                        # Draw coordinate axes (length = 0.1m = 10cm)
                        cv2.drawFrameAxes(display_image, self.camera_matrix, self.dist_coeffs,
                                        rvec, tvec, 0.1)
                        
                        # Add text overlay with detection info
                        info_text = [
                            f'Marker ID {self.marker_id} DETECTED',
                            f'Detections: {len(self.accumulated_transforms)}',
                            f'Time remaining: {remaining_time:.1f}s',
                            f'Camera -> ArUco: t=[{tvec[0][0]:.3f}, {tvec[1][0]:.3f}, {tvec[2][0]:.3f}]',
                            f'TF: OK'
                        ]
                        y_offset = 30
                        for i, text in enumerate(info_text):
                            cv2.putText(display_image, text, (10, y_offset + i * 25),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        
                        # Log every 20 detections to reduce spam
                        if len(self.accumulated_transforms) % 20 == 0:
                            self.get_logger().info(
                                f'Detections: {len(self.accumulated_transforms)} ({remaining_time:.1f}s remaining)'
                            )
                    except TransformException:
                        pass  # TF not ready yet, skip frame
                        
                        # Still show detection even if TF fails
                        cv2.aruco.drawDetectedMarkers(display_image, corners, ids)
                        cv2.drawFrameAxes(display_image, self.camera_matrix, self.dist_coeffs,
                                        rvec, tvec, 0.1)
                        
                        # Show error on image
                        error_text = [
                            f'Marker ID {self.marker_id} DETECTED',
                            f'Detections: {len(self.accumulated_transforms)}',
                            f'Time remaining: {remaining_time:.1f}s',
                            'ERROR: TF lookup failed!',
                            'Check: hand2eye_tf_publisher running?'
                        ]
                        y_offset = 30
                        for i, text in enumerate(error_text):
                            color = (0, 255, 0) if i < 3 else (0, 0, 255)
                            cv2.putText(display_image, text, (10, y_offset + i * 25),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            else:
                # Marker not detected - show status
                status_text = [
                    'Looking for ArUco marker...',
                    f'Marker ID: {self.marker_id}, Size: {self.marker_size}m',
                    f'Time remaining: {remaining_time:.1f}s'
                ]
                y_offset = 30
                for i, text in enumerate(status_text):
                    cv2.putText(display_image, text, (10, y_offset + i * 25),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                
                if ids is not None:
                    cv2.aruco.drawDetectedMarkers(display_image, corners, ids)
                    if self.marker_id not in ids:
                        cv2.putText(display_image, f'Found marker IDs: {ids.flatten()}', (10, y_offset + 75),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            
            # Show image
            if self.show_window and not self.detection_complete:
                cv2.imshow('ArUco Detection', display_image)
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
    
    def compute_final_transform(self):
        """Average accumulated detections and compute final transform"""
        if len(self.accumulated_transforms) == 0:
            self.get_logger().error('No detections accumulated! Make sure marker is visible.')
            return
        
        self.get_logger().info(f'Computing final transform from {len(self.accumulated_transforms)} detections...')
        
        # Average translations
        avg_translation = np.mean([t[:3, 3] for t in self.accumulated_transforms], axis=0)
        
        # Average rotations using quaternion averaging
        rotations = [t[:3, :3] for t in self.accumulated_transforms]
        quaternions = [R.from_matrix(rot).as_quat() for rot in rotations]
        avg_quat = np.mean(quaternions, axis=0)
        avg_quat = avg_quat / np.linalg.norm(avg_quat)  # Normalize
        avg_rotation = R.from_quat(avg_quat).as_matrix()
        
        # Build final transform matrix
        final_transform_matrix = np.eye(4)
        final_transform_matrix[:3, :3] = avg_rotation
        final_transform_matrix[:3, 3] = avg_translation
        
        # Create PoseStamped message for RVIZ visualization
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'lbr_link_0'
        
        pose_msg.pose.position.x = float(avg_translation[0])
        pose_msg.pose.position.y = float(avg_translation[1])
        pose_msg.pose.position.z = float(avg_translation[2])
        
        pose_msg.pose.orientation.x = float(avg_quat[0])
        pose_msg.pose.orientation.y = float(avg_quat[1])
        pose_msg.pose.orientation.z = float(avg_quat[2])
        pose_msg.pose.orientation.w = float(avg_quat[3])
        
        self.final_pose = pose_msg
        self.detection_complete = True
        
        # Print results
        self.print_results(final_transform_matrix, avg_translation, avg_quat)
        
        self.get_logger().info('Detection complete! Publishing to /aruco_detected_pose_new')
        self.get_logger().info('Copy values above into aruco_tf_publisher.cpp')
    
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
    
    def print_results(self, transform_matrix, translation, quaternion):
        """Print results in multiple formats"""
        rotation = R.from_matrix(transform_matrix[:3, :3])
        euler = rotation.as_euler('xyz', degrees=True)
        
        print('\n' + '=' * 70)
        print('Transform: lbr_link_0 → aruco_marker')
        print('\n--- TRANSLATION (meters) ---')
        print(f'  X: {translation[0]:+.6f}')
        print(f'  Y: {translation[1]:+.6f}')
        print(f'  Z: {translation[2]:+.6f}')
        print('\n--- ROTATION (quaternion) ---')
        print(f'  x: {quaternion[0]:+.6f}')
        print(f'  y: {quaternion[1]:+.6f}')
        print(f'  z: {quaternion[2]:+.6f}')
        print(f'  w: {quaternion[3]:+.6f}')
        print('\n--- ROTATION (Euler angles, degrees) ---')
        print(f'  Roll  (X): {euler[0]:+.2f}°')
        print(f'  Pitch (Y): {euler[1]:+.2f}°')
        print(f'  Yaw   (Z): {euler[2]:+.2f}°')
        print('\n// Translation')
        print(f'transform.transform.translation.x = {translation[0]:.6f};')
        print(f'transform.transform.translation.y = {translation[1]:.6f};')
        print(f'transform.transform.translation.z = {translation[2]:.6f};')
        print('\n// Rotation (quaternion)')
        print(f'tf2::Quaternion q;')
        print(f'q.setX({quaternion[0]:.6f});')
        print(f'q.setY({quaternion[1]:.6f});')
        print(f'q.setZ({quaternion[2]:.6f});')
        print(f'q.setW({quaternion[3]:.6f});')
        print('\ntransform.transform.rotation.x = q.x();')
        print('transform.transform.rotation.y = q.y();')
        print('transform.transform.rotation.z = q.z();')
        print('transform.transform.rotation.w = q.w();')
        print('\n' + '=' * 70)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectionNode()
    
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


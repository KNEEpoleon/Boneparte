#!/usr/bin/env python3
"""
ArUco Drill Pose Publisher Node

Subscribes to: /surgical_drill_pose (PoseArray in lbr_link_0 frame)
Publishes to: /aruco_drill_poses (PoseArray in aruco_marker frame)

Transforms drill poses from robot base frame to ArUco marker frame
for consumption by the Apple Vision Pro AR overlay system.
"""

import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformException

from geometry_msgs.msg import PoseArray, PoseStamped
from scipy.spatial.transform import Rotation as R
import numpy as np


class ArucoDrillPosePublisher(Node):
    def __init__(self):
        super().__init__('aruco_drill_pose_publisher')
        
        # TF2 buffer and listener for transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Publisher for transformed poses
        self.publisher = self.create_publisher(
            PoseArray, 
            '/aruco_drill_poses', 
            10
        )
        
        # Subscriber for drill poses from parasight/perception
        self.subscriber = self.create_subscription(
            PoseArray,
            '/surgical_drill_pose',
            self.pose_callback,
            10
        )
        
        self.get_logger().info('ArUco Drill Pose Publisher started')
        self.get_logger().info('Waiting for transforms and drill pose messages...')

    def pose_callback(self, msg: PoseArray):
        """
        Transform drill poses from lbr_link_0 frame to aruco_marker frame
        """
        try:
            # Get transform from lbr_link_0 to aruco_marker
            transform = self.tf_buffer.lookup_transform(
                'aruco_marker',      # target frame
                'lbr_link_0',        # source frame
                rclpy.time.Time(),   # latest available
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Extract transform components
            t = transform.transform.translation
            q = transform.transform.rotation
            translation = np.array([t.x, t.y, t.z])
            rotation = R.from_quat([q.x, q.y, q.z, q.w])
            
            # Create output PoseArray
            transformed_poses = PoseArray()
            transformed_poses.header.stamp = self.get_clock().now().to_msg()
            transformed_poses.header.frame_id = 'aruco_marker'
            
            # Transform each drill pose
            for pose in msg.poses:
                # Extract pose position and orientation
                pos = np.array([pose.position.x, pose.position.y, pose.position.z])
                ori = R.from_quat([
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w
                ])
                
                # Apply transform
                transformed_pos = rotation.apply(pos) + translation
                transformed_ori = rotation * ori
                
                # Create transformed pose
                transformed_pose = PoseStamped().pose
                transformed_pose.position.x = transformed_pos[0]
                transformed_pose.position.y = transformed_pos[1]
                transformed_pose.position.z = transformed_pos[2]
                
                q_transformed = transformed_ori.as_quat()
                transformed_pose.orientation.x = q_transformed[0]
                transformed_pose.orientation.y = q_transformed[1]
                transformed_pose.orientation.z = q_transformed[2]
                transformed_pose.orientation.w = q_transformed[3]
                
                transformed_poses.poses.append(transformed_pose)
            
            # Publish transformed poses
            self.publisher.publish(transformed_poses)
            
            self.get_logger().info(
                f'Transformed and published {len(transformed_poses.poses)} drill poses '
                f'from lbr_link_0 to aruco_marker frame'
            )
            
        except TransformException as e:
            self.get_logger().error(
                f'Failed to transform poses: {e}. '
                f'Make sure aruco_tf_publisher is running.'
            )
        except Exception as e:
            self.get_logger().error(f'Error in pose transformation: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDrillPosePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


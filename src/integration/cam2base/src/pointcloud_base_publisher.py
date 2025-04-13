#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
import numpy as np
from scipy.spatial.transform import Rotation as R

# Import your local helper functions
from ros2_numpy.point_cloud2 import (
    pointcloud2_to_array,
    array_to_pointcloud2,
    get_xyz_points
)

class PointCloudTransformer(Node):
    def __init__(self):
        super().__init__('pointcloud_base_publisher')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(PointCloud2, '/pointcloud_base', 10)
        self.subscriber = self.create_subscription(
            PointCloud2,
            '/camera/depth/color/points',
            self.callback,
            10
        )

    def callback(self, msg):
        try:
            # Get transform from camera_frame to lbr_link_0
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'lbr_link_0',
                'camera_frame',  # Verify that this matches your TF tree!
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # Convert PointCloud2 message to structured NumPy array
            cloud_np = pointcloud2_to_array(msg)

            # Extract XYZ points from structured array
            points_xyz = get_xyz_points(cloud_np, remove_nans=True)

            # Extract transform components
            t = transform.transform.translation
            q = transform.transform.rotation
            translation = np.array([t.x, t.y, t.z])
            rotation = R.from_quat([q.x, q.y, q.z, q.w])

            # Apply transform to point cloud
            transformed_points = rotation.apply(points_xyz) + translation

            # Update x, y, z fields in the original structured array
            cloud_np['x'][:len(transformed_points)] = transformed_points[:, 0]
            cloud_np['y'][:len(transformed_points)] = transformed_points[:, 1]
            cloud_np['z'][:len(transformed_points)] = transformed_points[:, 2]

            # Convert NumPy array back to PointCloud2 message
            transformed_msg = array_to_pointcloud2(
                cloud_np,
                stamp=self.get_clock().now().to_msg(),
                frame_id='lbr_link_0'
            )

            self.publisher.publish(transformed_msg)
            # self.get_logger().info(f"Published transformed PointCloud2 with {len(transformed_points)} points")

        except Exception as e:
            self.get_logger().error(f"Transform failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

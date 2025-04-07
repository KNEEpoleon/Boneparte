#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
import tf2_ros
import tf2_geometry_msgs  # Must be imported to register the conversions
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose


class CameraPoseTransformer(Node):
    def __init__(self):
        super().__init__('drill_pose_base_publisher')

        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publisher for transformed poses
        self.pose_publisher = self.create_publisher(PoseArray, '/surgical_drill_pose', 10)

        # Subscriber to the PoseArray from the camera
        self.pose_subscription = self.create_subscription(
            PoseArray,
            '/drill_pose_camera_frame',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg: PoseArray):
        try:
            # Get the transform
            transform = self.tf_buffer.lookup_transform(
                'lbr_link_0',  # target frame
                'camera_frame',  # source frame
                rclpy.time.Time(),  # latest available
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            transformed_array = PoseArray()
            transformed_array.header.frame_id = 'lbr_link_0'
            transformed_array.header.stamp = self.get_clock().now().to_msg()

            for pose in msg.poses:
                stamped_pose = PoseStamped()
                stamped_pose.header.frame_id = 'camera_frame'
                stamped_pose.header.stamp = self.get_clock().now().to_msg()
                stamped_pose.pose = pose

                # Transform the pose
                transformed_pose_stamped = do_transform_pose(stamped_pose, transform)
                transformed_array.poses.append(transformed_pose_stamped.pose)

            self.pose_publisher.publish(transformed_array)
            self.get_logger().info(f"Published {len(transformed_array.poses)} transformed poses.")

        except Exception as e:
            self.get_logger().error(f"Transform error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CameraPoseTransformer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

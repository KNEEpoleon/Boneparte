#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs

class CameraPoseTransformer(Node):
    def __init__(self):
        super().__init__('drill_pose_base_publisher')
        
        # Initialize the TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Publisher for transformed pose
        self.pose_publisher = self.create_publisher(PoseStamped, '/drill_pose_base_frame', 10)
        
        # Create a subscription to the /drill_pose_camera_frame topic
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/drill_pose_camera_frame',
            self.pose_callback,
            10
        )
    
    def pose_callback(self, msg):
        try:
            # Lookup the transform from camera_frame to base_link
            base_transform = self.tf_buffer.lookup_transform(
                'lbr_link_0',  # Target frame
                'camera_frame',  # Source frame
                rclpy.time.Time()  # Use the latest available transform
            )

            # Transform the pose from camera_frame to base_link
            transformed_pose = tf2_geometry_msgs.do_transform_pose(msg, base_transform)

            # Publish the transformed pose to /drill_pose_base_frame
            self.pose_publisher.publish(transformed_pose)

            # Log the transformed pose
            self.get_logger().info(f"Transformed Pose: x={transformed_pose.pose.position.x:.2f}, "
                                   f"y={transformed_pose.pose.position.y:.2f}, "
                                   f"z={transformed_pose.pose.position.z:.2f}")
        except tf2_ros.LookupException as ex:
            self.get_logger().error(f"Transform lookup failed: {ex}")
        except tf2_ros.ConnectivityException as ex:
            self.get_logger().error(f"Transform connectivity error: {ex}")
        except tf2_ros.ExtrapolationException as ex:
            self.get_logger().error(f"Transform extrapolation error: {ex}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraPoseTransformer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

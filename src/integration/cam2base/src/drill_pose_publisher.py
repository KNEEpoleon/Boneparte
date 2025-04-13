#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
import tf2_ros
import tf2_geometry_msgs  # Must be imported to register the conversions
from tf2_geometry_msgs import do_transform_pose


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
                stamped_pose = Pose()
                # stamped_pose.header.frame_id = 'camera_frame'
                # stamped_pose.header.stamp = self.get_clock().now().to_msg()
                stamped_pose = pose

                # Transform the pose
                transformed_pose_stamped = do_transform_pose(stamped_pose, transform)
                # transformed_pose_stamped.orientation[0] = 1.0 # this obviously won't work as the identity R will not allow for reorientation of Z!!
                # transformed_pose_stamped.orientation[1] = 0.0
                # transformed_pose_stamped.orientation[2] = 0.0
                # transformed_pose_stamped.orientation[3] = 0.0
                
                self.get_logger().info(f"\n\nTF pose stampled: {transformed_pose_stamped}")
                transformed_array.poses.append(transformed_pose_stamped)
                # [drill_pose_publisher.py-2] TF pose stampled: geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=-0.10903048778940018, y=0.029565822633570195, z=1.686571510661826), orientation=geometry_msgs.msg.Quaternion(x=-0.05757796911066912, y=0.2452627003257707, z=-0.7175222634022639, w=0.6493787699209871))


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

#!/usr/bin/env python3
# check the p12 p2 p3 vectors, how the normal is computed
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseArray, Point, Pose, PoseStamped, PointStamped
from std_msgs.msg import Empty
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_pose, do_transform_point
from std_msgs.msg import Int32
from transitions import Machine
import open3d as o3d

from parasight.segment_ui import SegmentAnythingUI
from parasight.registration import RegistrationPipeline
from parasight.utils import *

import time

from std_srvs.srv import Empty as EmptySrv
from functools import partial

from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation as R

from visualization_msgs.msg import Marker


class ParaSightHost(Node):
    states = ['start', 'await_surgeon_input', 'bring_manipulator', 'auto_reposition', 'segment_and_register', 'update_rviz', 'ready_to_drill', 'drill', 'finished']

    def __init__(self):
        super().__init__('parasight_host')
        
        # Create state machine
        self.machine = Machine(model=self, states=ParaSightHost.states, initial='start',
                               after_state_change='await_surgeon_input')
        
        # Add state transitions
        self.machine.add_transition(trigger='proceed_mission', source='await_surgeon_input', dest='bring_manipulator')
        self.machine.add_transition(trigger='complete_bring_manipulator', source='bring_manipulator', dest='auto_reposition')
        self.machine.add_transition(trigger='complete_auto_reposition', source='auto_reposition', dest='await_surgeon_input')
        self.machine.add_transition(trigger='annotate', source='await_surgeon_input', dest='segment_and_register')
        self.machine.add_transition(trigger='complete_segment_and_register', source='segment_and_register', dest='update_rviz')
        self.machine.add_transition(trigger='complete_map_update', source='update_rviz', dest='ready_to_drill')
        self.machine.add_transition(trigger='reset_mission', source='ready_to_drill', dest='await_surgeon_input')
        self.machine.add_transition(trigger='received_mission_pin', source='ready_to_drill', dest='drill')
        self.machine.add_transition(trigger='complete_drill', source='drill', dest='await_surgeon_input')
        self.machine.add_transition(trigger='complete_mission', source='await_surgeon_input', dest='finished')
        
        # State Data
        self.last_rgb_image = None
        self.last_depth_image = None
        self.last_cloud = None
        self.annotated_points = None
        self.need_to_register = True
        self.spam_counter = 0
        self.last_drill_pose_array = None
        self.last_drill_pose = None
        
        # D405 Intrinsics
        self.fx = 425.19189453125
        self.fy = 424.6562805175781
        self.cx = 422.978515625
        self.cy = 242.1155242919922

        # Set up tf listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.base_frame = 'lbr/link_0'
        self.ee_frame = 'lbr/link_ee'
        self.tool_frame = 'lbr/link_tool'
        self.camera_link_frame = 'camera_link'
        self.camera_frame = 'camera_depth_optical_frame'

        # Trigger Subscribers
        self.ui_trigger_subscription = self.create_subscription(
            Empty,
            '/trigger_host_ui',
            self.ui_trigger_callback,
            10)
        
        self.hard_reset_subscription = self.create_subscription(
            Empty,
            '/hard_reset_host',
            self.hard_reset_callback,
            10)
        
        # Data Subscribers
        self.rgb_image_subscription = self.create_subscription(
            Image,
            '/camera/color/image_rect_raw',
            self.rgb_image_callback,
            10)
        self.depth_image_subscription = self.create_subscription(
            Image,
            '/camera/aligned_depth_to_color/image_raw',
            self.depth_image_callback,
            10)
        self.pcd_subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/color/points',
            self.pcd_callback,
            10)
        
        # Publishers
        self.pcd_publisher = self.create_publisher(PointCloud2, '/processed_point_cloud', 10)
        self.pose_array_publisher = self.create_publisher(PoseArray, '/drill_pose_camera_frame', 10)
        self.marker_publisher = self.create_publisher(Marker, '/fitness_marker', 10)

        # Parameters
        self.declare_parameter('selected_bones', 'both')
        self.update_bones(self.get_parameter('selected_bones').value)
        self.add_on_set_parameters_callback(self.parameter_change_callback)

        # Load resources
        package_dir = get_package_share_directory('parasight')
        femur_cloud = o3d.io.read_point_cloud(package_dir + "/resource/femur_shell.ply")
        tibia_cloud = o3d.io.read_point_cloud(package_dir + "/resource/tibia_shell.ply")
        self.sources = {'femur': femur_cloud, 'tibia': tibia_cloud}
        self.colors = {'femur': [1, 0, 0], 'tibia': [0, 0, 1]}
        self.plan_path = package_dir + "/resource/plan_boneparte.yaml"
        self.plan = "plan1"

        # Interfaces
        self.regpipe = RegistrationPipeline()
        self.segmentation_ui = SegmentAnythingUI()
        print(f"Made the SAM object")
        self.bridge = CvBridge()

    # ============================================================================
    # STATE MACHINE CALLBACK
    # ============================================================================
    
    def await_surgeon_input(self):
        """Called after every state change to log current state."""
        self.get_logger().info(f'Current state: {self.state}')

    # ============================================================================
    # STATE ENTRY HANDLERS
    # ============================================================================
    
    def on_enter_start(self):
        """Entry handler for start state."""
        self.get_logger().info('System initialized and ready to begin surgery workflow')
    
    def on_enter_await_surgeon_input(self):
        """Entry handler for await_surgeon_input state."""
        self.get_logger().info('Awaiting surgeon input - ready for commands')
    
    def on_enter_bring_manipulator(self):
        """Entry handler for bring_manipulator state."""
        self.get_logger().info('Bringing manipulator from distant home to closer home location')
        # TODO: Implement manipulator bring command
        # This should move manipulator from far home position to near home position
        pass
    
    def on_enter_auto_reposition(self):
        """Entry handler for auto_reposition state."""
        self.get_logger().info('Entering auto_reposition - calling DINO bone extractor')
        # TODO: Call get_centroid from DINOBoneExtractor
        # This should analyze RGB image and return bone centroid for camera repositioning
        pass
    
    def on_enter_segment_and_register(self):
        """Entry handler for segment_and_register state."""
        self.get_logger().info('Entering segment_and_register - using SAM to segment bones')
        # Segmentation and registration is implemented via segment_with_ui
        masks, annotated_points, all_mask_points = self.segmentation_ui.segment_using_ui(
            self.last_rgb_image, self.bones
        )
        self.annotated_points = annotated_points
        self.get_logger().info(f'Annotated points: {self.annotated_points}')
        
        # Perform registration
        self.register_and_publish(annotated_points)
        
        # Transition to next state
        self.trigger('complete_segment_and_register')
    
    def on_enter_update_rviz(self):
        """Entry handler for update_rviz state."""
        self.get_logger().info('Updating RViz visualization')
        # TODO: Implement RViz update functionality
        # This should update visualization markers and displays
        # For now, automatically transition to next state
        self.trigger('complete_map_update')
    
    def on_enter_ready_to_drill(self):
        """Entry handler for ready_to_drill state."""
        self.get_logger().info('Plan registered - ready to receive drill command from surgeon')
    
    def on_enter_drill(self):
        """Entry handler for drill state."""
        self.get_logger().info('Executing drill command')
        # TODO: Implement actual drilling command to manipulator
        # This should send drilling commands based on received pin number
        pass
    
    def on_enter_finished(self):
        """Entry handler for finished state."""
        self.get_logger().info('Surgery workflow completed')

    # ============================================================================
    # CALLBACK METHODS
    # ============================================================================

    def hard_reset_callback(self, msg):
        """Reset the state machine to await_surgeon_input state."""
        self.get_logger().info('Hard reset requested - returning to await_surgeon_input')
        self.to_await_surgeon_input()

    def ui_trigger_callback(self, msg):
        """Handle UI trigger to start segmentation process."""
        self.get_logger().info(f'UI trigger received in state: {self.state}')
        
        if self.state == 'await_surgeon_input':
            self.trigger('annotate')
        else:
            self.get_logger().warn(f'UI trigger received but not in await_surgeon_input state (current: {self.state})')

    # ============================================================================
    # DATA CALLBACK METHODS
    # ============================================================================

    def rgb_image_callback(self, msg):
        """Callback for RGB image data."""
        rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        self.last_rgb_image = rgb_image

    def depth_image_callback(self, msg):
        """Callback for depth image data."""
        depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1") / 1000.0
        self.last_depth_image = depth_image
    
    def pcd_callback(self, msg):
        """Callback for point cloud data."""
        cloud = from_msg(msg)
        self.last_cloud = cloud

    # ============================================================================
    # PARAMETER METHODS
    # ============================================================================

    def update_bones(self, selected_bones):
        """Update self.bones based on the /selected_bones parameter."""
        if selected_bones == 'femur':
            self.bones = ['femur']
        elif selected_bones == 'tibia':
            self.bones = ['tibia']
        elif selected_bones == 'both':
            self.bones = ['femur', 'tibia']
        else:
            self.get_logger().warn(f"Unknown /selected_bones value: {selected_bones}, defaulting to empty list")
            self.bones = []
        self.get_logger().info(f"Updated bones to: {self.bones}")
    
    def parameter_change_callback(self, params):
        """Handle parameter changes."""
        for param in params:
            if param.name == 'selected_bones':
                self.update_bones(param.value)
        return SetParametersResult(successful=True)

    # ============================================================================
    # UTILITY METHODS
    # ============================================================================

    def add_depth(self, points):
        """Add depth information to 2D points."""
        new_points = []
        for point in points:
            x, y = point
            depth = average_depth(self.last_depth_image, y, x)
            new_points.append([x, y, depth])
        return new_points
    
    def pose_direction(self, annotated_points):
        """Dynamically compute theta based on bone positions."""
        femur_point_x = annotated_points[0][0]
        tibia_point_x = annotated_points[1][0]
        if femur_point_x > tibia_point_x:
            return 0
        else:
            return -np.pi

    # ============================================================================
    # REGISTRATION AND PUBLISHING METHODS (IMPLEMENTED)
    # ============================================================================

    def register_and_publish(self, points):
        """Segment, register, and publish bone point clouds and drill poses."""
        masks, annotated_points, all_mask_points = self.segmentation_ui.segment_using_points(
            self.last_rgb_image, points[0], points[1], self.bones
        )
        self.annotated_points = annotated_points
        annotated_points = self.add_depth(annotated_points)
        registered_clouds = []
        transforms = {}
        
        for i, bone in enumerate(self.bones):
            t0 = time.time()
            mask = masks[i]
            mask_points = all_mask_points[i]
            source = self.sources[bone]
            mask_points = self.add_depth(mask_points)
            transform, fitness = self.regpipe.register(
                mask, source, self.last_cloud, annotated_points, mask_points, bone=bone
            )
            t1 = time.time()
            self.get_logger().info(f'Registration time for {bone}: {t1 - t0}')
            source_cloud = source.voxel_down_sample(voxel_size=0.003)
            source_cloud.transform(transform)
            source_cloud.paint_uniform_color(self.colors[bone])
            registered_clouds.append(source_cloud)
            transforms[bone] = transform

        self.publish_point_cloud(registered_clouds)

        theta = self.pose_direction(annotated_points)
        drill_pose_array = self.compute_plan(transforms, theta=theta)
        drill_pose_array.header.frame_id = self.camera_frame
        drill_pose_array.header.stamp = self.get_clock().now().to_msg()
        self.pose_array_publisher.publish(drill_pose_array)

    def publish_point_cloud(self, clouds):
        """Combine and publish point clouds."""
        print(f"How many clouds are there: {len(clouds)}")
        combined_cloud = o3d.geometry.PointCloud()
        for c in clouds:
            print(f"in host py we have a cloud!")
            combined_cloud += c
        cloud_msg = to_msg(combined_cloud, frame_id=self.camera_frame)
        self.pcd_publisher.publish(cloud_msg)

    def compute_plan(self, transforms, theta=-np.pi/2):
        """Compute surgical drill points by transforming plan points with registration transforms."""
        drill_pose_array = PoseArray()

        parts = load_plan_points(self.plan_path, self.plan)
        self.get_logger().info(f"Updated parts plan to: {parts}")
        
        for bone, holes in parts.items():
            if bone not in self.bones:
                continue

            transform = transforms[bone]
            for hole_name, hole in holes.items():
                p1, p2, p3 = hole

                curr_theta = 0
                if bone == 'femur' and hole_name == 'hole2':
                    curr_theta = -np.pi/2
                elif bone == "femur" and hole_name == 'hole3':
                    curr_theta = -np.pi/2

                mesh = o3d.geometry.TriangleMesh()
                mesh.vertices = o3d.utility.Vector3dVector([p1, p2, p3])
                mesh.triangles = o3d.utility.Vector3iVector([[0, 1, 2]])
                mesh.compute_vertex_normals()
                normal = np.asarray(mesh.vertex_normals)[0]
                actual_normal = -normal
                z_axis = np.array([0, 0, 1])
                rotation_axis = np.cross(z_axis, actual_normal)
                rotation_axis /= np.linalg.norm(rotation_axis)
                angle = np.arccos(np.dot(z_axis, actual_normal) / (np.linalg.norm(z_axis) * np.linalg.norm(actual_normal)))
                Rot = o3d.geometry.get_rotation_matrix_from_axis_angle(rotation_axis * angle)

                T = np.eye(4)
                T[:3, :3] = Rot
                T[:3, 3] = p3

                default_plan = T
                default_plan_rotated = np.copy(default_plan)
                default_plan_rotated[:3, :3] = np.matmul(default_plan_rotated[:3, :3], R.from_euler('z', curr_theta).as_matrix())

                T = np.matmul(transform, default_plan_rotated)

                # Convert R to a quaternion
                Rot = T[:3, :3]
                r = R.from_matrix(Rot)
                quat = r.as_quat()  # Returns (x, y, z, w)

                p = T[:3, 3]

                pose = Pose()
                pose.position.x = p[0]
                pose.position.y = p[1]
                pose.position.z = p[2]

                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]

                self.get_logger().info(f"Bone: {bone} | hole: {hole_name} | Pose: {pose}")
                drill_pose_array.poses.append(pose)
        
        return drill_pose_array


def main(args=None):
    rclpy.init(args=args)
    node = ParaSightHost()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

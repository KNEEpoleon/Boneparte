#!/usr/bin/env python3
# check the p12 p2 p3 vectors, how the normal is computed
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseArray, Point, Pose, PoseStamped, PointStamped, Vector3
from std_msgs.msg import Empty, String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_pose, do_transform_point
from std_msgs.msg import Int32
from transitions import Machine
import open3d as o3d
import os


from parasight.segment_ui import SegmentAnythingUI
from parasight.registration import RegistrationPipeline
from parasight.dino_bone_extract import DINOBoneExtractor
from parasight.utils import *


import time
from datetime import datetime

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
                               after_state_change='after_state_change')
        
        # Add state transitions
        self.machine.add_transition(trigger='initialize', source='start', dest='await_surgeon_input')
        self.machine.add_transition(trigger='proceed_mission', source='await_surgeon_input', dest='bring_manipulator')
        self.machine.add_transition(trigger='complete_bring_manipulator', source='bring_manipulator', dest='auto_reposition')
        self.machine.add_transition(trigger='complete_auto_reposition', source='auto_reposition', dest='await_surgeon_input')
        self.machine.add_transition(trigger='annotate', source='await_surgeon_input', dest='segment_and_register')
        self.machine.add_transition(trigger='complete_segment_and_register', source='segment_and_register', dest='update_rviz')
        self.machine.add_transition(trigger='complete_map_update', source='update_rviz', dest='ready_to_drill')
        self.machine.add_transition(trigger='reset_mission', source='ready_to_drill', dest='await_surgeon_input')
        self.machine.add_transition(trigger='start_drill', source='ready_to_drill', dest='drill')
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

        # Trigger Subscribers (for state transitions)
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
        
        # Surgeon command subscribers (for await_surgeon_input state)
        self.proceed_mission_subscription = self.create_subscription(
            Empty,
            '/proceed_mission',
            self.proceed_mission_callback,
            10)
        
        self.annotate_subscription = self.create_subscription(
            Empty,
            '/annotate',
            self.annotate_callback,
            10)
        
        self.complete_mission_subscription = self.create_subscription(
            Empty,
            '/complete_mission',
            self.complete_mission_callback,
            10)
        
        self.reset_mission_subscription = self.create_subscription(
            Empty,
            '/reset_mission',
            self.reset_mission_callback,
            10)
        
        self.start_drill_subscription = self.create_subscription(
            Empty,
            '/start_drill',
            self.start_drill_callback,
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
        self.bone_centroid_publisher = self.create_publisher(PoseStamped, '/bone_centroid_camera_frame', 10)
        self.manipulator_command_publisher = self.create_publisher(String, '/manipulator_command', 10)
        self.reposition_vector_publisher = self.create_publisher(Vector3, '/error_recovery_direction', 10)
        # self.marker_publisher = self.create_publisher(Marker, '/fitness_marker', 10)

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
        
        checkpoint_path = "/ros_ws/src/perception/dinov3/checkpoints/dinov3_vitb16_pretrain_lvd1689m-73cec8be.pth"
        self.output_dir = "/ros_ws/src/perception/auto_reposition/"
        dinov3_path = "/ros_ws/src/perception/dinov3"
        codebook_path = "/ros_ws/src/perception/auto_reposition/fvd_bone_codebook.json"

        self.bone_extractor = DINOBoneExtractor(checkpoint_path=checkpoint_path, dinov3_path=dinov3_path, codebook_path=codebook_path, output_home=self.output_dir)
    

        # Initialize state machine after everything is set up
        self.initialize()

    # ============================================================================
    # STATE MACHINE CALLBACKS
    # ============================================================================

    def after_state_change(self):
        """Called automatically after every state transition."""
        self.get_logger().info(f'═══ State changed to: {self.state} ═══')

    # ============================================================================
    # STATE ENTRY/EXIT CALLBACKS
    # ============================================================================

    # def on_enter_start(self):
    #     """Entry handler for start state."""
    #     self.get_logger().info('System starting up...')
    #     # Note: Transition to await_surgeon_input happens automatically in __init__

    def on_enter_await_surgeon_input(self):
        """Entry handler for await_surgeon_input state - FULLY IMPLEMENTED.
        
        This state waits for surgeon commands via the following topics:
        - /proceed_mission → brings manipulator to position
        - /annotate → starts bone segmentation and registration
        - /complete_mission → finishes the surgery workflow
        """
        self.get_logger().info('═══ Awaiting surgeon input ═══')
        # Note: All transitions are handled by ROS callbacks in the ROS CALLBACK METHODS section

    def on_enter_bring_manipulator(self):
        """Entry handler for bring_manipulator state."""
        self.get_logger().info('Bringing manipulator to position...')
        # TODO: Implement manipulator positioning logic
        # For now, auto-complete for testing
        # self.get_logger().info('Auto-completing bring_manipulator (not implemented yet)')
        msg = String()
        msg.data = 'bring_home'
        self.manipulator_command_publisher.publish(msg)
        
        self.complete_bring_manipulator()

    def on_enter_auto_reposition(self):
        """Entry handler for auto_reposition state."""
        self.get_logger().info('Auto-repositioning...')
        # TODO: Implement auto-reposition logic
        # For now, auto-complete for testing
        if self.last_rgb_image is not None:
            os.makedirs(self.output_dir, exist_ok=True)
            # Get the current date and time
            now = datetime.now()
            # Format it into a concise string
            query_id = "query_" + now.strftime("%m-%d_%H:%M:%S") + ".png"
            cv2.imwrite(os.path.join(self.output_dir, query_id), self.last_rgb_image)

        detected_bone_msg = self.bone_extractor.get_centroid(os.path.join(self.output_dir, query_id))
        displacement_vector = detected_bone_msg['cluster_centroid']['vector']
        self.get_logger().info(f"Displacement vector: {displacement_vector}")
        msg = Vector3()
        msg.x = displacement_vector[0]
        msg.y = displacement_vector[1]
        msg.z = 0.1
        self.reposition_vector_publisher.publish(msg)

        self.complete_auto_reposition()

    def on_enter_segment_and_register(self):
        """Entry handler for segment_and_register state - FULLY IMPLEMENTED."""
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
        self.get_logger().info('Updating RViz visualization...')
        # TODO: Implement RViz update logic
        # For now, auto-complete for testing
        self.get_logger().info('Auto-completing update_rviz (not implemented yet)')
        self.complete_map_update()

    def on_enter_ready_to_drill(self):
        """Entry handler for ready_to_drill state."""
        self.get_logger().info('Ready to drill - awaiting mission pin...')

    def on_enter_drill(self):
        """Entry handler for drill state."""
        self.get_logger().info('Drilling in progress...')
        # TODO: Implement drilling logic
        # For now, auto-complete for testing
        self.get_logger().info('Auto-completing drill (not implemented yet)')
        self.complete_drill()

    def on_enter_finished(self):
        """Entry handler for finished state."""
        self.get_logger().info('Mission complete!')
        # TODO: Implement cleanup/finish logic

    # ============================================================================
    # ROS CALLBACK METHODS
    # ============================================================================

    def parameter_change_callback(self, params):
        """Handle parameter changes."""
        for param in params:
            if param.name == 'selected_bones':
                self.update_bones(param.value)
        return SetParametersResult(successful=True)

    def hard_reset_callback(self, msg):
        """Reset the state machine to initial state."""
        self.get_logger().warn('Hard reset triggered - returning to start state')
        self.to_start()

    def ui_trigger_callback(self, msg):
        """Handle UI trigger to start segmentation process."""
        self.get_logger().info(f'UI trigger received in state: {self.state}')
        
        if self.state == 'await_surgeon_input':
            self.trigger('annotate')
        else:
            self.get_logger().warn(f'UI trigger received but not in await_surgeon_input state (current: {self.state})')

    def proceed_mission_callback(self, msg):
        """Handle proceed_mission command - transitions to bring_manipulator."""
        self.get_logger().info(f'Proceed mission command received in state: {self.state}')
        
        if self.state == 'await_surgeon_input':
            self.get_logger().info('Triggering proceed_mission transition...')
            self.trigger('proceed_mission')
        else:
            self.get_logger().warn(f'Proceed mission command received but not in await_surgeon_input state (current: {self.state})')

    def annotate_callback(self, msg):
        """Handle annotate command - transitions to segment_and_register."""
        self.get_logger().info(f'Annotate command received in state: {self.state}')
        
        if self.state == 'await_surgeon_input':
            self.get_logger().info('Triggering annotate transition...')
            self.trigger('annotate')
        else:
            self.get_logger().warn(f'Annotate command received but not in await_surgeon_input state (current: {self.state})')

    def complete_mission_callback(self, msg):
        """Handle complete_mission command - transitions to finished."""
        self.get_logger().info(f'Complete mission command received in state: {self.state}')
        
        if self.state == 'await_surgeon_input':
            self.get_logger().info('Triggering complete_mission transition...')
            self.trigger('complete_mission')
        else:
            self.get_logger().warn(f'Complete mission command received but not in await_surgeon_input state (current: {self.state})')

    def reset_mission_callback(self, msg):
        """Handle reset_mission command - transitions from ready_to_drill to await_surgeon_input."""
        self.get_logger().info(f'Reset mission command received in state: {self.state}')
        
        if self.state == 'ready_to_drill':
            self.get_logger().info('Triggering reset_mission transition...')
            self.trigger('reset_mission')
        else:
            self.get_logger().warn(f'Reset mission command received but not in ready_to_drill state (current: {self.state})')

    def start_drill_callback(self, msg):
        """Handle start_drill command - transitions from ready_to_drill to drill."""
        self.get_logger().info(f'Start drill command received in state: {self.state}')
        
        if self.state == 'ready_to_drill':
            self.get_logger().info('Triggering start_drill transition...')
            self.trigger('start_drill')
        else:
            self.get_logger().warn(f'Start drill command received but not in ready_to_drill state (current: {self.state})')

    # ============================================================================
    # DATA CALLBACK METHODS
    # ============================================================================

    def rgb_image_callback(self, msg):
        """Callback for RGB image data - FULLY IMPLEMENTED."""
        rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        self.last_rgb_image = rgb_image

    def depth_image_callback(self, msg):
        """Callback for depth image data - FULLY IMPLEMENTED."""
        depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1") / 1000.0
        self.last_depth_image = depth_image
    
    def pcd_callback(self, msg):
        """Callback for point cloud data - FULLY IMPLEMENTED."""
        cloud = from_msg(msg)
        self.last_cloud = cloud

    # ============================================================================
    # UTILITY METHODS
    # ============================================================================

    def update_bones(self, selected_bones):
        """Update self.bones based on the /selected_bones parameter - FULLY IMPLEMENTED."""
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

    def add_depth(self, points):
        """Add depth information to 2D points - FULLY IMPLEMENTED."""
        new_points = []
        for point in points:
            x, y = point
            depth = average_depth(self.last_depth_image, y, x)
            new_points.append([x, y, depth])
        return new_points
    
    def pose_direction(self, annotated_points):
        """Dynamically compute theta based on bone positions - FULLY IMPLEMENTED."""
        femur_point_x = annotated_points[0][0]
        tibia_point_x = annotated_points[1][0]
        if femur_point_x > tibia_point_x:
            return 0
        else:
            return -np.pi

    # ============================================================================
    # REGISTRATION AND PUBLISHING METHODS
    # ============================================================================

    def register_and_publish(self, points):
        """Segment, register, and publish bone point clouds and drill poses - FULLY IMPLEMENTED."""
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
        """Combine and publish point clouds - FULLY IMPLEMENTED."""
        print(f"How many clouds are there: {len(clouds)}")
        combined_cloud = o3d.geometry.PointCloud()
        for c in clouds:
            print(f"in host py we have a cloud!")
            combined_cloud += c
        cloud_msg = to_msg(combined_cloud, frame_id=self.camera_frame)
        self.pcd_publisher.publish(cloud_msg)

    def compute_plan(self, transforms, theta=-np.pi/2):
        """Compute surgical drill points by transforming plan points with registration transforms - FULLY IMPLEMENTED."""
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


# ============================================================================
# MAIN ENTRY POINT
# ============================================================================

def main(args=None):
    """Main entry point for the ParaSight host node."""
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

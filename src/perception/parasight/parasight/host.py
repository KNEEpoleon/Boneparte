#!/usr/bin/env python3
# check the p12 p2 p3 vectors, how the normal is computed
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseArray, Point, Pose, PoseStamped, PointStamped, Vector3
from std_msgs.msg import Empty, String, String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_pose, do_transform_point
from std_msgs.msg import Int32
from transitions import Machine
import open3d as o3d
import os
import numpy as np

from parasight.segment_ui import SegmentAnythingUI
from parasight.registration import RegistrationPipeline
from parasight.dino_bone_extract import DINOBoneExtractor
from parasight.utils import *
from parasight.scripts.snapshot import SnapshotNode as snapshot_node


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
        self.machine.add_transition(trigger='hard_reset', source='*', dest='bring_manipulator')
        
        # State Data
        self.last_rgb_image = None
        self.last_depth_image = None
        self.last_depth_image_raw = None  # Keep original 16-bit depth data
        self.last_cloud = None
        self.annotated_points = None
        self.need_to_register = True
        self.spam_counter = 0
        self.last_drill_pose_array = None
        self.last_drill_pose = None
        
        # AVP annotations
        self.avp_annotations = None
        self.waiting_for_segmentation_approval = False
        self.pending_segmentation_data = None
        
        # Publisher for segmented image
        self.segmented_image_pub = self.create_publisher(Image, '/segmented_image', 10)
        
        # Services for segmentation approval
        self.approve_seg_service = self.create_service(EmptySrv, '/approve_segmentation', self.approve_segmentation_service)
        self.reject_seg_service = self.create_service(EmptySrv, '/reject_segmentation', self.reject_segmentation_service)
        
        # D405 Intrinsics from /camera_info
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
        self.camera_frame = 'camera_frame'

        # Trigger Subscribers (for state transitions)
        
        self.hard_reset_subscription = self.create_subscription(
            Empty,
            '/hard_reset_host',
            self.hard_reset_callback,
            10)
        
        # Surgeon command subscribers (for await_surgeon_input state)
        self.ui_trigger_subscription = self.create_subscription(
            Empty,
            '/trigger_host_ui',
            self.ui_trigger_callback,
            10)

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
        
        # AVP annotations subscription
        self.avp_annotations_subscription = self.create_subscription(
            String,
            '/avp_annotations',
            self.avp_annotations_callback,
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
        self.bone_centroid_publisher = self.create_publisher(PoseArray, '/bone_centroid_camera_frame', 10)
        self.manipulator_command_publisher = self.create_publisher(String, '/manipulator_command', 10)
        # self.reposition_vector_publisher = self.create_publisher(Vector3, '/error_recovery_direction', 10)
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
        self.auto_reposition_dir = "/ros_ws/src/perception/auto_reposition/"
        dinov3_path = "/ros_ws/src/perception/dinov3"
        codebook_path = "/ros_ws/src/perception/auto_reposition/fvd_bone_codebook.json"

        self.bone_extractor = DINOBoneExtractor(checkpoint_path=checkpoint_path, dinov3_path=dinov3_path, codebook_path=codebook_path, auto_reposition_dir=self.auto_reposition_dir)
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
        time.sleep(2.5)
        
        self.complete_bring_manipulator()

    def on_enter_auto_reposition(self):
        """Entry handler for auto_reposition state."""
        self.get_logger().info('Auto-repositioning...')
        # TODO: Implement auto-reposition logic
        # For now, auto-complete for testing
        while self.last_rgb_image is None or self.last_depth_image is None:
            time.sleep(0.5)
            self.get_logger().error("No RGB or depth image available")

        # Get the current date and time
        now = datetime.now()
        # Format it into a concise string
        query_id = "query_" + now.strftime("%m-%d_%H:%M:%S")
        self.bone_extractor.set_auto_resposition_query_dir(query_id)
        self.save_rgb_image(os.path.join(self.auto_reposition_dir, query_id, "rgb_snapshot.png"))
        self.save_depth_image(os.path.join(self.auto_reposition_dir, query_id, "depth_snapshot.png"))
        self.get_logger().info(f"Saved RGB image to {os.path.join(self.auto_reposition_dir, query_id, 'rgb_snapshot.png')}")
        detected_bone_msg = self.bone_extractor.get_centroid(os.path.join(self.auto_reposition_dir, query_id, "rgb_snapshot.png"))
        # Use live depth image (already in meters) instead of reloading from file
        bone_centroid_3d_camera_frame = self.pixel_to_3d_camera_frame(detected_bone_msg['cluster_centroid']['pixel_coords'], self.last_depth_image)
        bone_centroid_3d_camera_frame_msg = PoseArray()
        bone_centroid_3d_camera_frame_msg.header.frame_id = self.camera_frame
        bone_centroid_3d_camera_frame_msg.header.stamp = self.get_clock().now().to_msg()
        bone_centroid_pose = Pose()
        bone_centroid_pose.position.x = bone_centroid_3d_camera_frame[0]
        bone_centroid_pose.position.y = bone_centroid_3d_camera_frame[1]
        bone_centroid_pose.position.z = bone_centroid_3d_camera_frame[2]
        bone_centroid_3d_camera_frame_msg.poses.append(bone_centroid_pose)
        self.bone_centroid_publisher.publish(bone_centroid_3d_camera_frame_msg)
        # image_centroid_3d_camera_frame = self.pixel_to_3d_camera_frame(np.array([self.cx, self.cy]), query_id)
        # displacement_vector = bone_centroid_3d_camera_frame - image_centroid_3d_camera_frame
        # displacement_vector_msg = Vector3()
        # displacement_vector_msg.x = displacement_vector[0]
        # displacement_vector_msg.y = displacement_vector[1]
        # displacement_vector_msg.z = displacement_vector[2]
        # self.get_logger().info(f"Displacement vector: {displacement_vector_msg}")
        # self.reposition_vector_publisher.publish(displacement_vector_msg)

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
        self.trigger('hard_reset')

    def avp_annotations_callback(self, msg):
        """Store AVP annotations for use in segmentation"""
        try:
            import json
            self.avp_annotations = json.loads(msg.data)
            self.get_logger().info(f'Received AVP annotations: {self.avp_annotations}')
        except Exception as e:
            self.get_logger().error(f'Failed to parse AVP annotations: {e}')

    def all_systems_ready(self, dummy=True):
        if dummy:
            return True
            
        try:
            # Check tf transform
            self.tf_buffer.lookup_transform('world', 'camera_color_optical_frame', rclpy.time.Time())
            
            # Check required nodes
            node_names = [node.split('/')[-1] for node in self.get_node_names()]
            if 'io_node' not in node_names or 'registration_node' not in node_names:
                return False
                
            return True
            
        except TransformException:
            return False

    def publish_state(self):
        self.get_logger().info(f'Current state: {self.state}')

    def ui_trigger_callback(self, msg):
        print(f"\n! The self state is: {self.state}")

        self.get_logger().info('UI trigger received')
        if self.state == 'ready':
            self.trigger('start_parasight')
            
            # Check if we have AVP annotations, otherwise use UI
            if self.avp_annotations is not None:
                self.get_logger().info('Using AVP annotations for segmentation')
                # Convert pixel coordinates to the format expected by segment_using_points
                femur_point = tuple(self.avp_annotations[0])  # [x, y]
                tibia_point = tuple(self.avp_annotations[1])  # [x, y]
                result = self.segmentation_ui.segment_using_points(
                    self.last_rgb_image, femur_point, tibia_point, self.bones)
                masks, annotated_points, all_mask_points, segmented_image = result
                
                # Store segmentation data and wait for AVP approval
                self.pending_segmentation_data = {
                    'masks': masks,
                    'annotated_points': annotated_points,
                    'all_mask_points': all_mask_points,
                    'segmented_image': segmented_image
                }
                self.waiting_for_segmentation_approval = True
                
                # Send segmented image to AVP via TCP server
                self.get_logger().info('Segmentation complete, publishing segmented image')
                self.publish_segmented_image(segmented_image)
                
                # Clear AVP annotations after use
                self.avp_annotations = None
            else:
                self.get_logger().info('Using UI for segmentation')
                masks, annotated_points, all_mask_points = self.segmentation_ui.segment_using_ui(self.last_rgb_image, self.bones) # Blocking call
                self.annotated_points = annotated_points
                print(f"\n The annotated points are: {self.annotated_points}")
                # For UI mode, proceed immediately
                self.register_and_publish(annotated_points)
                self.trigger('drill_complete')

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
    
    def approve_segmentation(self):
        """Called by TCP server when AVP accepts segmentation"""
        if self.waiting_for_segmentation_approval and self.pending_segmentation_data:
            self.get_logger().info('Segmentation approved by AVP, proceeding with registration')
            # Use the already-computed segmentation data instead of re-segmenting
            masks = self.pending_segmentation_data['masks']
            annotated_points = self.pending_segmentation_data['annotated_points']
            all_mask_points = self.pending_segmentation_data['all_mask_points']
            
            self.annotated_points = annotated_points
            print(f"\n The annotated points are: {self.annotated_points}")
            
            # Proceed with registration using the stored masks
            self.register_and_publish_with_masks(masks, annotated_points, all_mask_points)
            # Note: AVP workflow needs integration with FSM - trigger removed during merge fix
            self.waiting_for_segmentation_approval = False
            self.pending_segmentation_data = None
        else:
            self.get_logger().warn('approve_segmentation called but not waiting for approval')
    
    def reject_segmentation(self):
        """Called by TCP server when AVP rejects segmentation"""
        if self.waiting_for_segmentation_approval:
            self.get_logger().info('Segmentation rejected by AVP, clearing data')
            self.waiting_for_segmentation_approval = False
            self.pending_segmentation_data = None
            # Note: AVP workflow needs integration with FSM - trigger removed during merge fix
        else:
            self.get_logger().warn('reject_segmentation called but not waiting for approval')
    
    def publish_segmented_image(self, segmented_image):
        """Publish segmented image for TCP server to send to AVP"""
        try:
            # Convert numpy array to ROS Image message
            if len(segmented_image.shape) == 3 and segmented_image.shape[2] == 4:
                # RGBA
                image_msg = self.bridge.cv2_to_imgmsg(segmented_image, encoding='rgba8')
            else:
                # RGB
                image_msg = self.bridge.cv2_to_imgmsg(segmented_image, encoding='rgb8')
            
            self.segmented_image_pub.publish(image_msg)
            self.get_logger().info('Published segmented image')
        except Exception as e:
            self.get_logger().error(f'Failed to publish segmented image: {e}')
    
    def approve_segmentation_service(self, request, response):
        """Service callback for approving segmentation"""
        self.approve_segmentation()
        self.get_logger().info('Approved segmentation')
        return response
    
    def reject_segmentation_service(self, request, response):
        """Service callback for rejecting segmentation"""
        self.reject_segmentation()
        self.get_logger().warn('Rejected segmentation')
        return response

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
        # Keep original 16-bit depth data (in millimeters per ROS2 16UC1 standard)
        depth_image_raw = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        self.last_depth_image_raw = depth_image_raw
        
        # Convert millimeters to meters for processing (float64)
        # 16UC1 encoding is standard millimeters for RealSense depth images
        depth_image = depth_image_raw.astype(np.float64) / 1000.0
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

    def pixel_to_3d_camera_frame(self, pixel_coords, depth_image_meters):
        """Convert pixel coordinates to 3D coordinates in the camera frame.
        
        Args:
            pixel_coords: [u, v] pixel coordinates
            depth_image_meters: Depth image in meters (from self.last_depth_image)
        
        Returns:
            np.array([x, y, z]) in meters in the optical frame coordinate system
        """
        z = average_depth(depth_image_meters, pixel_coords[1], pixel_coords[0])
        x = (pixel_coords[0] - self.cx) * z / self.fx
        y = (pixel_coords[1] - self.cy) * z / self.fy
        return np.array([x, y, z])
    # ============================================================================
    # REGISTRATION AND PUBLISHING METHODS
    # ============================================================================

    def register_and_publish_with_masks(self, masks, annotated_points, all_mask_points):
        """Registration and publishing using pre-computed masks (for AVP workflow)"""
        # Cache annotated points
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
            transform, fitness = self.regpipe.register(mask, source, self.last_cloud, annotated_points, mask_points, bone=bone)
            t1 = time.time()
            self.get_logger().info(f'\n\nRegistration time for {bone}: {t1 - t0}, \nfitness: {fitness}')
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

    def register_and_publish(self, points):
        """Segment, register, and publish bone point clouds and drill poses - FULLY IMPLEMENTED."""
        # self.pause_tracking() # Pause tracking while registering to improve performance
        masks, annotated_points, all_mask_points, _ = self.segmentation_ui.segment_using_points(
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
            self.get_logger().info(f'\n\nRegistration time for {bone}: {t1 - t0}, \nfitness: {fitness}')
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
                elif bone == "tibia" and hole_name == 'hole4':
                    curr_theta = -np.pi/4-np.pi/2
                elif bone == "tibia" and hole_name == 'hole5':
                    curr_theta = np.pi +np.pi/12-np.pi/2
#                    curr_theta = np.pi/6-np.pi/4-np.pi/2
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


    def save_rgb_image(self, filepath):
        if self.last_rgb_image is None:
            self.get_logger().warn("No RGB image available to save")
            return
        cv2.imwrite(filepath, self.last_rgb_image)
        self.get_logger().info(f"Saved RGB image: {filepath}")

    def save_depth_image(self, filepath):
        """Save depth image preserving 16-bit millimeter values from RealSense.
        
        Saves the raw 16UC1 depth data (millimeters) for archival/debugging purposes.
        Note: For 3D computations, use self.last_depth_image (meters) directly.
        """
        if self.last_depth_image_raw is None:
            self.get_logger().warn("No depth image available to save")
            return
        
        # Save as 16-bit PNG preserving original millimeter values (16UC1 standard)
        cv2.imwrite(filepath, self.last_depth_image_raw)
        self.get_logger().info(f"Saved 16-bit depth PNG (millimeters): {filepath}")

    @staticmethod
    def load_depth_image(filepath):
        """Load depth image from saved files."""
        base_path = os.path.splitext(filepath)[0]
        
        # # Try to load 16-bit PNG first (most compatible lossless format)
        # png_path = base_path + "_16bit.png"
        # if os.path.exists(png_path):
        #     return cv2.imread(png_path, cv2.IMREAD_UNCHANGED)
        
        # Fall back to original path if it exists
        if os.path.exists(filepath):
            return cv2.imread(filepath, cv2.IMREAD_UNCHANGED)
        
        raise FileNotFoundError(f"No depth image found at {filepath}")


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

#!/usr/bin/env python3
"""
ParaSight Host State Machine Implementation

Simplified state machine for surgical workflow with clearer separation of concerns.
Planning/manipulation nodes are called via services/topics, not embedded in states.
"""

import rclpy
from rclpy.node import Node
from transitions import Machine
from std_msgs.msg import Empty, Int32, String
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image, PointCloud2
import open3d as o3d

from parasight.segment_ui import SegmentAnythingUI
from parasight.registration import RegistrationPipeline
from ament_index_python.packages import get_package_share_directory


class ParaSightHost(Node):
    """
    Updated state machine with auto_reposition:
    - start: Initial state
    - auto_reposition: CV-based robot positioning using DINO features
    - waiting: Waiting for doctor commands (annotate or drill mission)
    - segmenting: User annotates bone regions
    - registering: Automatic registration and obstacle update
    - drilling: Command sent to manipulator, waiting for completion
    - finished: Surgery complete
    """
    
    states = [
        'start',
        'auto_reposition',   # CV-based robot positioning (DINO features)
        'waiting',           # Combined doc_verify and ready_to_drill
        'segmenting',        # Combined annotate and register (user interaction part)
        'registering',       # Automatic registration after segmentation
        'drilling',          # Drilling state (waits for manipulator)
        'finished'
    ]

    def __init__(self):
        super().__init__('parasight_host')
        
        # ===== State Machine Setup =====
        self.machine = Machine(
            model=self,
            states=ParaSightHost.states,
            initial='start',
            auto_transitions=False
        )
        
        # Define transitions
        self.machine.add_transition(
            trigger='begin_surgery',
            source='start',
            dest='auto_reposition',
            after='on_enter_auto_reposition'
        )
        
        self.machine.add_transition(
            trigger='complete_auto_reposition',
            source='auto_reposition',
            dest='waiting',
            after='on_enter_waiting'
        )
        
        self.machine.add_transition(
            trigger='request_annotation',
            source='waiting',
            dest='segmenting',
            after='on_enter_segmenting'
        )
        
        self.machine.add_transition(
            trigger='complete_segmentation',
            source='segmenting',
            dest='registering',
            after='on_enter_registering'
        )
        
        self.machine.add_transition(
            trigger='complete_registration',
            source='registering',
            dest='waiting',
            after='on_enter_waiting'
        )
        
        self.machine.add_transition(
            trigger='request_drill',
            source='waiting',
            dest='drilling',
            before='on_enter_drilling'
        )
        
        self.machine.add_transition(
            trigger='complete_drill',
            source='drilling',
            dest='waiting',
            after='on_enter_waiting'
        )
        
        self.machine.add_transition(
            trigger='end_surgery',
            source='waiting',
            dest='finished',
            after='on_enter_finished'
        )
        
        # ===== Data Storage =====
        self.last_rgb_image = None
        self.last_depth_image = None
        self.last_cloud = None
        self.annotated_points = None
        self.current_drill_mission = None
        self.registered_poses = None
        self.auto_reposition_complete = False
        self.computed_robot_position = None
        
        # ===== Publishers =====
        self.registration_status_pub = self.create_publisher(
            String,
            '/registration',
            10
        )
        
        self.pose_array_publisher = self.create_publisher(
            PoseArray,
            '/surgical_drill_pose',
            10
        )
        
        self.pcd_publisher = self.create_publisher(
            PointCloud2,
            '/processed_point_cloud',
            10
        )
        
        self.robot_position_cmd_pub = self.create_publisher(
            PoseArray,  # or whatever message type for robot positioning
            '/cmd/robot_position',
            10
        )
        
        # ===== Subscribers =====
        # External command subscribers
        self.annotate_cmd_sub = self.create_subscription(
            Empty,
            '/cmd/request_annotation',
            self.cmd_request_annotation_callback,
            10
        )
        
        self.drill_cmd_sub = self.create_subscription(
            Int32,
            '/cmd/drill_mission',
            self.cmd_drill_mission_callback,
            10
        )
        
        self.drill_complete_sub = self.create_subscription(
            Empty,
            '/manipulation/drill_complete',
            self.manipulation_drill_complete_callback,
            10
        )
        
        self.reposition_complete_sub = self.create_subscription(
            Empty,
            '/manipulation/reposition_complete',
            self.manipulation_reposition_complete_callback,
            10
        )
        
        self.end_surgery_sub = self.create_subscription(
            Empty,
            '/cmd/end_surgery',
            self.cmd_end_surgery_callback,
            10
        )
        
        # Sensor data subscribers
        self.rgb_image_subscription = self.create_subscription(
            Image,
            '/camera/color/image_rect_raw',
            self.rgb_image_callback,
            10
        )
        
        self.depth_image_subscription = self.create_subscription(
            Image,
            '/camera/aligned_depth_to_color/image_raw',
            self.depth_image_callback,
            10
        )
        
        self.pcd_subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/color/points',
            self.pcd_callback,
            10
        )
        
        # ===== Interfaces =====
        package_dir = get_package_share_directory('parasight')
        femur_cloud = o3d.io.read_point_cloud(package_dir + "/resource/femur_shell.ply")
        tibia_cloud = o3d.io.read_point_cloud(package_dir + "/resource/tibia_shell.ply")
        self.sources = {'femur': femur_cloud, 'tibia': tibia_cloud}
        self.colors = {'femur': [1, 0, 0], 'tibia': [0, 0, 1]}
        self.plan_path = package_dir + "/resource/plan_config_v2.yaml"
        
        self.regpipe = RegistrationPipeline()
        self.segmentation_ui = SegmentAnythingUI()
        
        self.get_logger().info('ParaSight Host initialized in START state')
        self.get_logger().info('Call /cmd/begin_surgery to start workflow')
    
    # ========================================
    # STATE CALLBACKS (on_enter_*)
    # ========================================
    
    def on_enter_auto_reposition(self):
        """
        Auto-reposition state - use DINO features and CV to determine optimal robot position
        """
        self.get_logger().info('====================================')
        self.get_logger().info('STATE: AUTO_REPOSITION')
        self.get_logger().info('Analyzing scene with DINO features...')
        self.get_logger().info('Computing optimal robot position...')
        self.get_logger().info('====================================')
        
        if self.last_rgb_image is None or self.last_cloud is None:
            self.get_logger().error('No sensor data available for auto-repositioning!')
            self.get_logger().error('Skipping auto-reposition, moving to waiting state...')
            self.complete_auto_reposition()
            return
        
        try:
            # Perform CV-based robot positioning
            robot_position = self.compute_optimal_robot_position(
                self.last_rgb_image, 
                self.last_depth_image, 
                self.last_cloud
            )
            
            if robot_position is not None:
                # Publish positioning command to manipulation stack
                self.robot_position_cmd_pub.publish(robot_position)
                self.computed_robot_position = robot_position
                
                self.get_logger().info('Robot positioning command sent to manipulation stack')
                self.get_logger().info('Waiting for repositioning completion...')
                
                # Wait for manipulation to signal completion via /manipulation/reposition_complete
                # The callback will trigger complete_auto_reposition()
            else:
                self.get_logger().error('Failed to compute optimal robot position')
                self.complete_auto_reposition()  # Skip to next state
                
        except Exception as e:
            self.get_logger().error(f'Auto-reposition failed: {e}')
            self.complete_auto_reposition()  # Skip to next state
    
    def on_enter_waiting(self):
        """
        Waiting state - system is idle, waiting for:
        - Annotation request from doctor
        - Drill mission command
        - End surgery command
        """
        self.get_logger().info('====================================')
        self.get_logger().info('STATE: WAITING')
        self.get_logger().info('Ready for commands:')
        self.get_logger().info('  - /cmd/request_annotation')
        self.get_logger().info('  - /cmd/drill_mission')
        self.get_logger().info('  - /cmd/end_surgery')
        self.get_logger().info('====================================')
        
        # Publish idle status
        self.registration_status_pub.publish(String(data='idle'))
    
    def on_enter_segmenting(self):
        """
        Segmentation state - user annotates bone regions using UI
        """
        self.get_logger().info('====================================')
        self.get_logger().info('STATE: SEGMENTING')
        self.get_logger().info('Opening segmentation UI...')
        self.get_logger().info('====================================')
        
        if self.last_rgb_image is None or self.last_cloud is None:
            self.get_logger().error('No image data available for segmentation!')
            self.get_logger().error('Returning to waiting state...')
            self.complete_segmentation()  # Auto-transition back
            return
        
        try:
            # Call segmentation UI (blocking)
            masks, annotated_points, mask_points = self.segmentation_ui.register(self.last_rgb_image)
            self.annotated_points = {'points': annotated_points, 'mask_points': mask_points, 'masks': masks}
            
            self.get_logger().info('Segmentation complete!')
            self.complete_segmentation()  # Trigger transition to registering
            
        except Exception as e:
            self.get_logger().error(f'Segmentation failed: {e}')
            self.complete_segmentation()  # Return to waiting
    
    def on_enter_registering(self):
        """
        Registration state - automatically register bones and update obstacles
        """
        self.get_logger().info('====================================')
        self.get_logger().info('STATE: REGISTERING')
        self.get_logger().info('Performing automatic registration...')
        self.get_logger().info('====================================')
        
        # Publish registering status
        self.registration_status_pub.publish(String(data='registering'))
        
        if self.annotated_points is None:
            self.get_logger().error('No annotated points available!')
            self.registration_status_pub.publish(String(data='failed'))
            self.complete_registration()
            return
        
        try:
            # Perform registration
            transforms, registered_clouds = self.perform_registration(
                self.annotated_points['points'],
                self.annotated_points['mask_points'],
                self.annotated_points['masks']
            )
            
            # Compute drill poses
            drill_poses = self.compute_drill_poses(transforms)
            self.registered_poses = drill_poses
            
            # Publish results
            self.pose_array_publisher.publish(drill_poses)
            self.publish_point_cloud(registered_clouds)
            
            # Publish complete status
            self.registration_status_pub.publish(String(data='complete'))
            
            self.get_logger().info(f'Registration complete! {len(drill_poses.poses)} drill poses available')
            
            # Transition back to waiting
            self.complete_registration()
            
        except Exception as e:
            self.get_logger().error(f'Registration failed: {e}')
            self.registration_status_pub.publish(String(data='failed'))
            self.complete_registration()
    
    def on_enter_drilling(self):
        """
        Drilling state - command sent to manipulator, waiting for completion
        
        NOTE: This state does NOT control the manipulator directly.
        It assumes manipulation node is subscribed to /cmd/drill_mission
        and will publish to /manipulation/drill_complete when done.
        """
        self.get_logger().info('====================================')
        self.get_logger().info('STATE: DRILLING')
        self.get_logger().info(f'Drill mission: pin index {self.current_drill_mission}')
        self.get_logger().info('Waiting for manipulation node to complete...')
        self.get_logger().info('====================================')
        
        # Mission command already published by cmd_drill_mission_callback
        # Just wait for /manipulation/drill_complete
    
    def on_enter_finished(self):
        """
        Finished state - surgery complete
        """
        self.get_logger().info('====================================')
        self.get_logger().info('STATE: FINISHED')
        self.get_logger().info('Surgery workflow complete!')
        self.get_logger().info('====================================')
        
        # Cleanup
        self.annotated_points = None
        self.registered_poses = None
        self.current_drill_mission = None
        self.auto_reposition_complete = False
        self.computed_robot_position = None
    
    # ========================================
    # COMMAND CALLBACKS (External triggers)
    # ========================================
    
    def cmd_request_annotation_callback(self, msg):
        """Doctor requests bone annotation"""
        self.get_logger().info('Received annotation request')
        if self.state == 'waiting':
            self.request_annotation()
        else:
            self.get_logger().warn(f'Cannot annotate in state: {self.state}')
    
    def cmd_drill_mission_callback(self, msg):
        """Doctor requests drilling at specific pin index"""
        pin_index = msg.data
        self.get_logger().info(f'Received drill mission: pin {pin_index}')
        
        if self.state == 'waiting':
            self.current_drill_mission = pin_index
            
            # Publish mission to manipulation stack
            # (Manipulation node should subscribe to /cmd/drill_mission)
            self.get_logger().info(f'Forwarding drill command to manipulation stack')
            
            self.request_drill()
        else:
            self.get_logger().warn(f'Cannot drill in state: {self.state}')
    
    def manipulation_drill_complete_callback(self, msg):
        """Manipulation node signals drilling complete"""
        self.get_logger().info('Received drill complete signal from manipulation')
        
        if self.state == 'drilling':
            self.complete_drill()
        else:
            self.get_logger().warn(f'Unexpected drill complete in state: {self.state}')
    
    def manipulation_reposition_complete_callback(self, msg):
        """Manipulation node signals repositioning complete"""
        self.get_logger().info('Received reposition complete signal from manipulation')
        
        if self.state == 'auto_reposition':
            self.complete_auto_reposition()
        else:
            self.get_logger().warn(f'Unexpected reposition complete in state: {self.state}')
    
    def cmd_end_surgery_callback(self, msg):
        """Doctor ends surgery"""
        self.get_logger().info('Received end surgery command')
        
        if self.state == 'waiting':
            self.end_surgery()
        else:
            self.get_logger().warn(f'Cannot end surgery in state: {self.state}')
    
    # ========================================
    # SENSOR DATA CALLBACKS
    # ========================================
    
    def rgb_image_callback(self, msg):
        """Store latest RGB image"""
        # TODO: Convert ROS Image to OpenCV format
        self.last_rgb_image = msg
    
    def depth_image_callback(self, msg):
        """Store latest depth image"""
        self.last_depth_image = msg
    
    def pcd_callback(self, msg):
        """Store latest point cloud"""
        # TODO: Convert to Open3D point cloud
        self.last_cloud = msg
    
    # ========================================
    # HELPER FUNCTIONS (Placeholders)
    # ========================================
    
    def compute_optimal_robot_position(self, rgb_image, depth_image, point_cloud):
        """
        Use DINO features and computer vision to determine optimal robot positioning
        
        TODO: Implement DINO feature extraction and CV pipeline
        
        Args:
            rgb_image: RGB camera image
            depth_image: Depth camera image  
            point_cloud: 3D point cloud data
            
        Returns:
            PoseArray: Optimal robot position command for manipulation stack
        """
        self.get_logger().info('Extracting DINO features from scene...')
        
        # TODO: Implement actual DINO feature extraction
        # TODO: Analyze scene geometry and bone positions
        # TODO: Compute optimal robot base position and orientation
        # TODO: Consider workspace constraints and collision avoidance
        
        # Placeholder - return a sample robot position
        pose_array = PoseArray()
        pose_array.header.frame_id = 'lbr_link_0'
        pose_array.header.stamp = self.get_clock().now().to_msg()
        
        # Add sample pose (this should be computed from DINO analysis)
        from geometry_msgs.msg import Pose
        sample_pose = Pose()
        sample_pose.position.x = 0.5  # Example position
        sample_pose.position.y = 0.0
        sample_pose.position.z = 0.0
        sample_pose.orientation.w = 1.0  # Identity rotation
        
        pose_array.poses.append(sample_pose)
        
        self.get_logger().info('Computed optimal robot position (placeholder)')
        return pose_array
    
    def perform_registration(self, annotated_points, mask_points, masks):
        """
        Perform bone registration
        
        TODO: Implement actual registration logic
        """
        self.get_logger().info('Performing registration...')
        
        # Placeholder - implement actual registration
        transforms = {}
        registered_clouds = []
        
        return transforms, registered_clouds
    
    def compute_drill_poses(self, transforms):
        """
        Compute drill poses from registration transforms
        
        TODO: Implement actual pose computation
        """
        self.get_logger().info('Computing drill poses...')
        
        # Placeholder
        pose_array = PoseArray()
        pose_array.header.frame_id = 'lbr_link_0'
        pose_array.header.stamp = self.get_clock().now().to_msg()
        
        return pose_array
    
    def publish_point_cloud(self, clouds):
        """
        Publish registered point clouds for visualization
        
        TODO: Implement actual publishing
        """
        pass


def main(args=None):
    rclpy.init(args=args)
    node = ParaSightHost()
    
    # Start the workflow
    node.begin_surgery()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


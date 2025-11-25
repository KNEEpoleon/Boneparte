#!/usr/bin/env python3
"""
Unified TCP Server Node - Handles all AVP communication on 3 ports

Port 5000 (Control Channel): Commands and FSM state updates
Port 5001 (Drill Poses Channel): High-frequency drill pose stream (10Hz)
Port 5002 (Images Channel): Large image transfers (annotation, segmentation)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
from std_srvs.srv import Empty as EmptySrv
from surgical_robot_planner.srv import SelectPose, RobotCommand, ClearObstacle
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray
from controller_manager_msgs.srv import SwitchController
from cv_bridge import CvBridge
import socket
import base64
import cv2
import json
import threading
import time


class UnifiedTcpServerNode(Node):
    def __init__(self):
        super().__init__('unified_tcp_server')
        self.get_logger().info('Unified TCP Server Node started')

        # ============================================================================
        # ROS PUBLISHERS & SUBSCRIBERS
        # ============================================================================

        # FSM command publishers
        self.annotate_pub = self.create_publisher(Empty, '/annotate', 10)
        self.proceed_mission_pub = self.create_publisher(Empty, '/proceed_mission', 10)
        self.reset_mission_pub = self.create_publisher(Empty, '/reset_mission', 10)
        self.hard_reset_pub = self.create_publisher(Empty, '/hard_reset_host', 10)
        
        # Publisher for AVP annotations
        self.annotations_pub = self.create_publisher(String, '/avp_annotations', 10)

        # Service clients
        self.select_pose_client = self.create_client(SelectPose, '/select_pose')
        self.robot_command_client = self.create_client(RobotCommand, '/robot_command')
        self.approve_seg_client = self.create_client(EmptySrv, '/approve_segmentation')
        self.reject_seg_client = self.create_client(EmptySrv, '/reject_segmentation')
        self.clear_obstacle_client = self.create_client(ClearObstacle, '/clear_obstacle')
        self.switch_controller_client = self.create_client(SwitchController, '/lbr/controller_manager/switch_controller')

        # Data subscriptions
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/color/image_rect_raw',
            self.image_callback,
            10)
        
        self.segmented_image_subscription = self.create_subscription(
            Image,
            '/segmented_image',
            self.segmented_image_callback,
            10)
        
        # Drill poses subscription (from avp_tcp_server.py)
        self.drill_poses_subscription = self.create_subscription(
            PoseArray,
            '/aruco_drill_poses',
            self.drill_poses_callback,
            10)

        # ============================================================================
        # DATA STORAGE
        # ============================================================================
        
        self.bridge = CvBridge()
        self.last_rgb_image = None
        self.rgb_image_sent = False  # Track if RGB image was already sent for current annotation
        self.pending_segmented_image = None
        self.segmented_image_sent = False  # Track if segmented image was already sent
        
        # Drill poses
        self.latest_drill_poses = None
        self.drill_poses_lock = threading.Lock()
        self.last_drill_pose_count = 0  # Track drill pose count changes

        # Annotation response handling
        self.annotation_response = None
        self.annotation_lock = threading.Lock()
        self.waiting_for_annotation = False
        self.annotation_start_time = None
        
        # ============================================================================
        # PORT 5000: CONTROL CHANNEL (Commands + FSM State)
        # ============================================================================
        
        self.control_server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.control_server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.control_server_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.control_server_sock.bind(('0.0.0.0', 5000))
        self.control_server_sock.listen(1)
        self.control_server_sock.setblocking(False)
        
        self.control_client_sock = None
        self.control_client_addr = None
        
        self.get_logger().info('Control channel listening on port 5000')
        
        # ============================================================================
        # PORT 5001: DRILL POSES CHANNEL
        # ============================================================================
        
        self.poses_server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.poses_server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.poses_server_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.poses_server_sock.bind(('0.0.0.0', 5001))
        self.poses_server_sock.listen(1)
        self.poses_server_sock.setblocking(False)
        
        self.poses_client_sock = None
        self.poses_client_addr = None

        self.get_logger().info('Drill poses channel listening on port 5001')

        # ============================================================================
        # PORT 5002: IMAGES CHANNEL
        # ============================================================================
        
        self.images_server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.images_server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.images_server_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.images_server_sock.bind(('0.0.0.0', 5002))
        self.images_server_sock.listen(1)
        self.images_server_sock.setblocking(False)

        self.images_client_sock = None
        self.images_client_addr = None

        self.get_logger().info('Images channel listening on port 5002')

        # ============================================================================
        # TIMERS
        # ============================================================================
        
        # Poll all three sockets at 10Hz
        self.control_timer = self.create_timer(0.1, self.poll_control_socket)
        self.images_timer = self.create_timer(0.1, self.poll_images_socket)
        
        # Send drill poses at 10Hz
        self.poses_timer = self.create_timer(0.1, self.send_drill_poses)

    # ============================================================================
    # ROS CALLBACKS
    # ============================================================================

    def image_callback(self, msg):
        """Store the latest RGB image for annotation"""
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
            self.last_rgb_image = rgb_image
        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')
    
    def segmented_image_callback(self, msg):
        """Receive segmented image from ParaSight and send to AVP"""
        try:
            segmented_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.pending_segmented_image = segmented_image
            self.get_logger().info('Received segmented image from ParaSight')
            # Send immediately to AVP (only if not already sent)
            if not self.segmented_image_sent:
                self.send_segmented_image_to_avp()
                self.segmented_image_sent = True  # Mark as sent
        except Exception as e:
            self.get_logger().error(f'Failed to process segmented image: {e}')
    
    def drill_poses_callback(self, msg):
        """Store latest drill poses"""
        with self.drill_poses_lock:
            self.latest_drill_poses = msg
        
        self.get_logger().debug(f'Received {len(msg.poses)} drill poses in aruco_marker frame')

    # ============================================================================
    # PORT 5000: CONTROL CHANNEL LOGIC
    # ============================================================================
    
    def poll_control_socket(self):
        """Poll control channel for connections and commands"""
        # Accept new connections if no client connected
        if self.control_client_sock is None:
            try:
                client, addr = self.control_server_sock.accept()
            except BlockingIOError:
                return  # No incoming connection
            
            self.control_client_sock = client
            self.control_client_addr = addr
            self.control_client_sock.setblocking(False)
            self.get_logger().info(f'Control client connected from {addr}')
            return
        
        # Check for annotation timeout
        if self.waiting_for_annotation:
            if time.time() - self.annotation_start_time > 60:  # 60 second timeout
                with self.annotation_lock:
                    self.waiting_for_annotation = False
                self.get_logger().error("Timeout waiting for annotation response")
                return
        
        # Receive commands from AVP
        try:
            data = self.control_client_sock.recv(1024)
        except BlockingIOError:
            return  # No data available
        except Exception as e:
            self.get_logger().error(f'Control channel error: {e}')
            self.disconnect_control_client()
            return

        if not data:
            # Client disconnected gracefully
            self.get_logger().info('Control client disconnected')
            self.disconnect_control_client()
            return
        
        # Process command
        message = data.decode('utf-8').strip()
        if message:
            self.get_logger().info(f'Received command: "{message}"')
            self.handle_command(message)
            
            # Send acknowledgment
            try:
                self.control_client_sock.sendall(b'acknowledged\n')
            except (BrokenPipeError, ConnectionResetError, OSError) as e:
                self.get_logger().warn(f'Control client disconnected while sending ack: {e}')
                self.disconnect_control_client()
    
    def disconnect_control_client(self):
        """Disconnect control channel client"""
        try:
            if self.control_client_sock:
                self.control_client_sock.close()
        except:
            pass
        self.control_client_sock = None
        self.control_client_addr = None
        self.get_logger().info('Control channel waiting for new connection...')

    # ============================================================================
    # PORT 5001: DRILL POSES CHANNEL LOGIC
    # ============================================================================
    
    def send_drill_poses(self):
        """Send drill poses at 10Hz on Port 5001"""
        # Accept new connections if no client connected
        if self.poses_client_sock is None:
            try:
                client, addr = self.poses_server_sock.accept()
            except BlockingIOError:
                return  # No incoming connection
            
            self.poses_client_sock = client
            self.poses_client_addr = addr
            self.poses_client_sock.setblocking(False)
            self.get_logger().info(f'Drill poses client connected from {addr}')
            return
        
        # Send latest drill poses if available
        with self.drill_poses_lock:
            if self.latest_drill_poses is not None and len(self.latest_drill_poses.poses) > 0:
                try:
                    message = self.format_poses_message(self.latest_drill_poses)
                    self.poses_client_sock.sendall(message.encode('utf-8'))
                    
                    # Only log when drill pose count changes
                    current_count = len(self.latest_drill_poses.poses)
                    if current_count != self.last_drill_pose_count:
                        self.get_logger().info(f'Sent {current_count} drill poses to AVP')
                        self.last_drill_pose_count = current_count
                
                except (BrokenPipeError, ConnectionResetError, OSError) as e:
                    self.get_logger().warn(f'Drill poses client disconnected: {e}')
                    self.disconnect_poses_client()
        
        # Check for client messages (heartbeat, etc.)
        if self.poses_client_sock is not None:
            try:
                data = self.poses_client_sock.recv(1024)
                if not data:
                    # Client disconnected gracefully
                    self.get_logger().info('Drill poses client disconnected')
                    self.disconnect_poses_client()
                else:
                    # Echo acknowledgment (optional)
                    message = data.decode('utf-8').strip()
                    self.get_logger().debug(f'Received from drill poses client: {message}')
            except BlockingIOError:
                pass  # No data available
            except Exception as e:
                self.get_logger().error(f'Drill poses channel error: {e}')
                self.disconnect_poses_client()
    
    def format_poses_message(self, poses: PoseArray) -> str:
        """Format poses as: POSES|x,y,z,qx,qy,qz,qw|x,y,z,qx,qy,qz,qw|..."""
        pose_strings = []
        for pose in poses.poses:
            pose_str = (
                f"{pose.position.x:.6f},"
                f"{pose.position.y:.6f},"
                f"{pose.position.z:.6f},"
                f"{pose.orientation.x:.6f},"
                f"{pose.orientation.y:.6f},"
                f"{pose.orientation.z:.6f},"
                f"{pose.orientation.w:.6f}"
            )
            pose_strings.append(pose_str)
        
        return "POSES|" + "|".join(pose_strings) + "\n"
    
    def disconnect_poses_client(self):
        """Disconnect drill poses channel client"""
        try:
            if self.poses_client_sock:
                self.poses_client_sock.close()
        except:
            pass
        self.poses_client_sock = None
        self.poses_client_addr = None

    # ============================================================================
    # PORT 5002: IMAGES CHANNEL LOGIC
    # ============================================================================
    
    def poll_images_socket(self):
        """Poll images channel for connections and annotation responses"""
        # Accept new connections if no client connected
        if self.images_client_sock is None:
            try:
                client, addr = self.images_server_sock.accept()
            except BlockingIOError:
                return  # No incoming connection
            
            self.images_client_sock = client
            self.images_client_addr = addr
            self.images_client_sock.setblocking(False)
            self.get_logger().info(f'Images client connected from {addr}')
            return
        
        # Receive annotation responses from AVP
        try:
            data = self.images_client_sock.recv(1024)
        except BlockingIOError:
            return  # No data available
        except Exception as e:
            self.get_logger().error(f'Images channel error: {e}')
            self.disconnect_images_client()
            return

        if not data:
            # Client disconnected gracefully
            self.get_logger().info('Images client disconnected')
            self.disconnect_images_client()
            return
        
        # Process annotation response
        message = data.decode('utf-8').strip()
        if message:
            self.get_logger().info(f'Received from images channel: "{message}"')
            
            # Check if this is an annotation response
            if message.startswith("ANNOTATIONS:"):
                self.get_logger().info('Processing AVP annotation response...')
                self.handle_annotation_response(message)

            # Send acknowledgment
            try:
                self.images_client_sock.sendall(b'acknowledged\n')
            except (BrokenPipeError, ConnectionResetError, OSError) as e:
                self.get_logger().warn(f'Images client disconnected while sending ack: {e}')
                self.disconnect_images_client()

    def send_image_to_avp(self):
        """Send RGB image to AVP for annotation on Port 5002"""
        if self.images_client_sock is None:
            self.get_logger().error('No images client connection available')
            return
        
        if self.last_rgb_image is None:
            self.get_logger().error("No image available for annotation - camera topic not publishing yet")
            self.get_logger().error("Make sure /camera/color/image_rect_raw is publishing")
            return
        
        # Check if image was already sent for this annotation cycle
        if self.rgb_image_sent:
            self.get_logger().debug('RGB image already sent for this annotation cycle, skipping')
            return
        
        try:
            # Convert image to JPEG and encode as base64
            self.get_logger().info('Converting image to JPEG and base64...')
            _, buffer = cv2.imencode('.jpg', self.last_rgb_image, [cv2.IMWRITE_JPEG_QUALITY, 85])
            image_base64 = base64.b64encode(buffer).decode('utf-8')
            self.get_logger().info(f'Base64 encoding complete, size: {len(image_base64)} chars')
            
            # Send image to AVP
            image_message = f"IMAGE:{image_base64}\n"
            self.get_logger().info(f'Preparing to send message, total size: {len(image_message)} chars')
            
            # Temporarily make socket blocking for large send
            original_blocking = self.images_client_sock.getblocking()
            self.images_client_sock.setblocking(True)
            try:
                self.get_logger().info('Sending image data to AVP...')
                self.images_client_sock.sendall(image_message.encode('utf-8'))
                self.get_logger().info('Sent image to AVP for annotation')
                self.rgb_image_sent = True  # Mark as sent
            finally:
                # Restore original blocking state
                if self.images_client_sock is not None:
                    self.images_client_sock.setblocking(original_blocking)
            
            # Set up waiting state
            with self.annotation_lock:
                self.annotation_response = None
                self.waiting_for_annotation = True
                self.annotation_start_time = time.time()
            
            self.get_logger().info('Waiting for annotation response...')
                
        except (BrokenPipeError, ConnectionResetError, OSError) as e:
            self.get_logger().error(f'Images client disconnected while sending image: {e}')
            self.disconnect_images_client()
        except Exception as e:
            self.get_logger().error(f'Failed to send image: {e}')
    
    def send_segmented_image_to_avp(self):
        """Send segmented image to AVP for review on Port 5002"""
        if self.images_client_sock is None:
            self.get_logger().error('No images client connection available')
            return
        
            if self.pending_segmented_image is None:
                self.get_logger().error('No pending segmented image to send')
                return
            
        try:
            segmented_image = self.pending_segmented_image
            
            # Convert to RGB if needed
            if len(segmented_image.shape) == 3 and segmented_image.shape[2] == 4:
                segmented_image = cv2.cvtColor(segmented_image, cv2.COLOR_RGBA2RGB)
            elif len(segmented_image.shape) == 3 and segmented_image.shape[2] == 3:
                # Already RGB, ensure correct color order
                segmented_image = cv2.cvtColor(segmented_image, cv2.COLOR_BGR2RGB)
            
            # Convert image to JPEG and encode as base64
            self.get_logger().info('Converting segmented image to JPEG and base64...')
            _, buffer = cv2.imencode('.jpg', segmented_image, [cv2.IMWRITE_JPEG_QUALITY, 85])
            image_base64 = base64.b64encode(buffer).decode('utf-8')
            self.get_logger().info(f'Base64 encoding complete, size: {len(image_base64)} chars')
            
            # Send segmented image to AVP
            image_message = f"SEGMENTED_IMAGE:{image_base64}\n"
            self.get_logger().info(f'Preparing to send segmented image, total size: {len(image_message)} chars')
            
            # Temporarily make socket blocking for large send
            original_blocking = self.images_client_sock.getblocking()
            self.images_client_sock.setblocking(True)
            try:
                self.get_logger().info('Sending segmented image to AVP...')
                self.images_client_sock.sendall(image_message.encode('utf-8'))
                self.get_logger().info('Sent segmented image to AVP for review')
            finally:
                # Restore original blocking state
                if self.images_client_sock is not None:
                    self.images_client_sock.setblocking(original_blocking)
                self.pending_segmented_image = None  # Clear after sending
                
        except (BrokenPipeError, ConnectionResetError, OSError) as e:
            self.get_logger().error(f'Images client disconnected while sending segmented image: {e}')
            self.disconnect_images_client()
        except Exception as e:
            self.get_logger().error(f'Failed to send segmented image: {e}')
    
    def disconnect_images_client(self):
        """Disconnect images channel client"""
        try:
            if self.images_client_sock:
                self.images_client_sock.close()
        except:
            pass
        self.images_client_sock = None
        self.images_client_addr = None

    # ============================================================================
    # COMMAND HANDLING (Port 5000)
    # ============================================================================

    def handle_command(self, command: str):
        """Handle commands received on control channel"""
        if command == "annotate":
            self.handle_annotate_command()
        elif command == "proceed_mission":
            self.handle_proceed_mission()
        elif command == "reset_mission":
            self.handle_reset_mission()
        elif command == "restart":
            self.hard_reset_pub.publish(Empty())
            self.get_logger().info('Published to /hard_reset_host')
        elif command == "KILLALL":
            self.handle_emergency_stop()
        elif command == "accept":
            self.handle_accept_segmentation()
        elif command == "reject":
            self.handle_reject_segmentation()
        elif command == "robot_home":
            self.call_robot_command_service("home")
        elif command == "robot_away":
            self.call_robot_command_service("away")
        elif command.startswith("drill_"):
            _, bone, hole = command.split('_')
            try:
                if bone == "femur":
                    if hole == "1":
                        self.call_select_pose_service(0)
                    elif hole == "2":
                        self.call_select_pose_service(1)
                    elif hole == "3":
                        self.call_select_pose_service(2)
                elif bone == "tibia":
                    if hole == "1":
                        self.call_select_pose_service(3)
                    elif hole == "2":
                        self.call_select_pose_service(4)
            except:
                self.get_logger().warn(f"Invalid pose index in command: {command}")
        elif command.startswith("clear_"):
            self.get_logger().info(f'🗑️  Clear command received: "{command}"')
            _, bone, hole = command.split('_')
            try:
                if bone == "femur":
                    if hole == "1":
                        self.call_clear_obstacle_service(0)
                    elif hole == "2":
                        self.call_clear_obstacle_service(1)
                    elif hole == "3":
                        self.call_clear_obstacle_service(2)
                elif bone == "tibia":
                    if hole == "1":
                        self.call_clear_obstacle_service(3)
                    elif hole == "2":
                        self.call_clear_obstacle_service(4)
            except Exception as e:
                self.get_logger().warn(f"Invalid pose index in clear command: {command}, error: {e}")
        else:
            self.get_logger().warn(f'Unknown command: "{command}"')

    def handle_annotate_command(self):
        """Handle annotate command - send image to AVP on images channel"""
        # Reset the RGB image sent flag for new annotation cycle
        self.rgb_image_sent = False
        self.send_image_to_avp()

    def handle_annotation_response(self, message: str):
        """Handle annotation response from AVP"""
        try:
            self.get_logger().info(f'Full annotation message: "{message}"')
            
            # Extract JSON part after "ANNOTATIONS:"
            json_str = message[12:]  # Remove "ANNOTATIONS:" prefix
            self.get_logger().info(f'Extracted JSON string: "{json_str}"')
            
            annotations = json.loads(json_str)
            self.get_logger().info(f'Parsed annotations successfully: {annotations}')
            
            with self.annotation_lock:
                self.annotation_response = annotations
                self.waiting_for_annotation = False
            
            self.get_logger().info(f'AVP annotations stored. Annotations: {annotations}')
            
            # Process the annotations immediately
            self.process_annotations_and_continue()
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON decode error: {e}. Raw JSON: "{json_str}"')
        except Exception as e:
            self.get_logger().error(f'Failed to parse annotation response: {e}. Full message: "{message}"')

    def process_annotations_and_continue(self):
        """Process received annotations and continue with original pipeline"""
        try:
            # Convert normalized coordinates to pixel coordinates
            height, width = self.last_rgb_image.shape[:2]
            
            pixel_coords = []
            for annotation in self.annotation_response:
                x_norm = annotation['x']  # 0.0 to 1.0
                y_norm = annotation['y']  # 0.0 to 1.0
                
                x_pixel = int(x_norm * width)
                y_pixel = int(y_norm * height)
                pixel_coords.append([x_pixel, y_pixel])
            
            self.get_logger().info(f'Converted annotations to pixel coordinates: {pixel_coords}')
            
            # Publish the annotations for ParaSight host to use
            annotations_msg = String()
            annotations_msg.data = json.dumps(pixel_coords)
            self.annotations_pub.publish(annotations_msg)
            self.get_logger().info('Published AVP annotations to ParaSight host')
            
            time.sleep(0.2)
            
            # Now trigger the FSM annotate transition (will generate segmentation)
            self.annotate_pub.publish(Empty())
            self.get_logger().info('Published to /annotate with AVP annotations')
            # Segmented image will be sent via callback when ParaSight publishes it
            
        except Exception as e:
            self.get_logger().error(f'Failed to process annotations: {e}')
    
    def handle_accept_segmentation(self):
        """Handle accept command from AVP - proceed with drill pose computation"""
        self.get_logger().info('Segmentation accepted by AVP, calling approve service')
        
        # Reset flag for next segmentation
        self.segmented_image_sent = False
        self.pending_segmented_image = None
        
        if not self.approve_seg_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service /approve_segmentation not available')
            return
        
        request = EmptySrv.Request()
        future = self.approve_seg_client.call_async(request)
        self.get_logger().info('Called approve_segmentation service')
    
    def handle_reject_segmentation(self):
        """Handle reject command from AVP - reset FSM to await_surgeon_input for new annotations"""
        self.get_logger().info('Segmentation rejected by AVP')
        
        # Reset flag for next segmentation
        self.segmented_image_sent = False
        self.pending_segmented_image = None
        
        # Call reject service first to clean up ParaSight state
        if not self.reject_seg_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service /reject_segmentation not available')
        else:
            request = EmptySrv.Request()
            future = self.reject_seg_client.call_async(request)
            self.get_logger().info('Called reject_segmentation service')
        
        # Reset FSM back to await_surgeon_input state
        self.reset_mission_pub.publish(Empty())
        self.get_logger().info('Published to /reset_mission - FSM returning to await_surgeon_input')
    
    def handle_proceed_mission(self):
        """Handle proceed_mission command - move robot home and advance FSM through auto-reposition"""
        self.get_logger().info('Proceed mission command received')
        
        # Publish to FSM to start proceed_mission transition
        # FSM will: await_surgeon_input -> bring_manipulator -> auto_reposition -> await_surgeon_input
        self.proceed_mission_pub.publish(Empty())
        self.get_logger().info('Published to /proceed_mission - FSM starting bring_manipulator -> auto_reposition')

    def handle_reset_mission(self):
        """Handle reset_mission command - return FSM to bring_manipulator state for reannotation"""
        self.get_logger().info('Reset mission command received')
        
        # Publish to FSM to trigger reset_mission transition
        # FSM will: * -> bring_manipulator
        self.reset_mission_pub.publish(Empty())
        self.get_logger().info('Published to /reset_mission - FSM transitioning to bring_manipulator')

    def handle_emergency_stop(self):
        """Handle emergency stop command - stop robot controllers via ros2_control"""
        self.get_logger().error('EMERGENCY STOP ACTIVATED - Stopping robot controllers')
        
        try:
            # Create service request to stop joint_trajectory_controller
            request = SwitchController.Request()
            request.stop_controllers = ['joint_trajectory_controller']
            request.start_controllers = []
            request.strictness = 1  # BEST_EFFORT
            request.start_asap = False
            request.timeout = rclpy.duration.Duration(seconds=0.0).to_msg()
            
            # Call service asynchronously (fire-and-forget)
            future = self.switch_controller_client.call_async(request)
            
            self.get_logger().info('Emergency stop command sent - controller deactivation requested')
            
        except Exception as e:
            self.get_logger().error(f'Error during emergency stop: {e}')

    # ============================================================================
    # SERVICE CALLS
    # ============================================================================

    def call_select_pose_service(self, index: int):
        """Call service to select drill pose"""
        if not self.select_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service /select_pose not available')
            return

        request = SelectPose.Request()
        request.index = index

        future = self.select_pose_client.call_async(request)
        future.add_done_callback(self.handle_service_response)
        self.get_logger().info(f"Drilling bone index: {index}")

    def handle_service_response(self, future):
        """Handle service response"""
        try:
            response = future.result()
            self.get_logger().info(f'Service responded: {response}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def call_robot_command_service(self, command: str):
        """Call the robot command service to move robot to home or away position"""
        if not self.robot_command_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service /robot_command not available')
            return

        request = RobotCommand.Request()
        request.command = command  # "home" or "away"

        future = self.robot_command_client.call_async(request)
        future.add_done_callback(self.handle_robot_command_response)
        self.get_logger().info(f"Commanding robot to move to: {command}")

    def handle_robot_command_response(self, future):
        """Handle response from robot command service"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Robot command succeeded: {response.message}')
            else:
                self.get_logger().error(f'Robot command failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Robot command service call failed: {e}')

    def call_clear_obstacle_service(self, pose_index: int):
        """Call the clear obstacle service to remove a pin obstacle from planning stack"""
        if not self.clear_obstacle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service /clear_obstacle not available')
            return

        request = ClearObstacle.Request()
        request.pose_index = pose_index

        future = self.clear_obstacle_client.call_async(request)
        future.add_done_callback(self.handle_clear_obstacle_response)
        self.get_logger().info(f"Clearing obstacle at pose_index: {pose_index}")

    def handle_clear_obstacle_response(self, future):
        """Handle response from clear obstacle service"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Clear obstacle succeeded: {response.message}')
            else:
                self.get_logger().error(f'Clear obstacle failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Clear obstacle service call failed: {e}')

    # ============================================================================
    # CLEANUP
    # ============================================================================

    def __del__(self):
        """Cleanup on node destruction"""
        try:
            if self.control_client_sock:
                self.control_client_sock.close()
            self.control_server_sock.close()
        except:
            pass
        
        try:
            if self.poses_client_sock:
                self.poses_client_sock.close()
            self.poses_server_sock.close()
        except:
            pass
        
        try:
            if self.images_client_sock:
                self.images_client_sock.close()
            self.images_server_sock.close()
        except:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = UnifiedTcpServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

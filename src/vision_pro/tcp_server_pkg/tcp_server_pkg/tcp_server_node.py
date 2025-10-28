#!/usr/bin/env python3
# ros2_tcp_server_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
from surgical_robot_planner.srv import SelectPose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import socket
import base64
import cv2
import json
import threading
import time
import subprocess

class TcpServerNode(Node):
    def __init__(self):
        super().__init__('tcp_server_node')
        self.get_logger().info('ROS2 TCP Server Node started')

        # One-shot publisher
        self.start_pub = self.create_publisher(Empty, '/trigger_host_ui', 10)
        self.stop_pub = self.create_publisher(Empty, '/hard_reset_host', 10)
        
        # Publisher for AVP annotations
        self.annotations_pub = self.create_publisher(String, '/avp_annotations', 10)

        # Service client
        self.select_pose_client = self.create_client(SelectPose, '/select_pose')

        # Image subscription for annotation
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/color/image_rect_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.last_rgb_image = None

        # Annotation response handling
        self.annotation_response = None
        self.annotation_lock = threading.Lock()
        self.waiting_for_annotation = False
        self.annotation_start_time = None

        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.server_sock.bind(('0.0.0.0', 5000))
        self.server_sock.listen(1)
        self.server_sock.setblocking(False)

        self.client_sock = None
        self.client_addr = None

        self.timer = self.create_timer(0.1, self.poll_socket)

    def image_callback(self, msg):
        """Store the latest RGB image for annotation"""
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
            self.last_rgb_image = rgb_image
        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')

    def poll_socket(self):
        if self.client_sock is None:
            try:
                client, addr = self.server_sock.accept()
            except BlockingIOError:
                return
            self.client_sock = client
            self.client_addr = addr
            self.client_sock.setblocking(False)
            self.get_logger().info(f'Accepted TCP connection from {addr}')
            return

        # Check for annotation timeout
        if self.waiting_for_annotation:
            if time.time() - self.annotation_start_time > 60:  # 60 second timeout
                with self.annotation_lock:
                    self.waiting_for_annotation = False
                self.get_logger().error("Timeout waiting for annotation response")
                return
        
        try:
            data = self.client_sock.recv(1024)
        except BlockingIOError:
            return

        if not data:
            print("\n\n\n", data, flush = True)
            self.get_logger().info('Client disconnected')
            self.client_sock.close()
            self.client_sock = None
            self.client_addr = None
        else:
            message = data.decode('utf-8').strip()
            if message:
                self.get_logger().info(f'Received raw data from AVP: "{message}"')
                self.get_logger().info(f'Message length: {len(message)} characters')
                self.get_logger().info(f'Message starts with ANNOTATIONS: {message.startswith("ANNOTATIONS:")}')
                
                # Check if this is an annotation response
                if message.startswith("ANNOTATIONS:"):
                    self.get_logger().info('Processing AVP annotation response...')
                    self.handle_annotation_response(message)
                else:
                    self.get_logger().info(f'Processing non-annotation command: "{message}"')
                    self.handle_command(message)

                try:
                    self.client_sock.sendall(b'acknowledged\n')
                except Exception as e:
                    self.get_logger().error(f'Failed to send acknowledgment: {e}')

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
            
            self.get_logger().info(f'AVP annotations stored and waiting flag cleared. Annotations: {annotations}')
            
            # Process the annotations immediately
            self.process_annotations_and_continue()
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON decode error: {e}. Raw JSON: "{json_str}"')
        except Exception as e:
            self.get_logger().error(f'Failed to parse annotation response: {e}. Full message: "{message}"')

    def handle_annotate_command(self):
        """Handle annotation command by sending image to AVP and waiting for response"""
        try:
            # Check if we have an image
            if self.last_rgb_image is None:
                raise Exception("No image available for annotation")
            
            # Convert image to PNG and encode as base64
            self.get_logger().info('Converting image to PNG and base64...')
            _, buffer = cv2.imencode('.png', self.last_rgb_image)
            image_base64 = base64.b64encode(buffer).decode('utf-8')
            self.get_logger().info(f'Base64 encoding complete, size: {len(image_base64)} chars')
            
            # Send image to AVP
            image_message = f"IMAGE:{image_base64}\n"
            self.get_logger().info(f'Preparing to send message, total size: {len(image_message)} chars')
            
            # Temporarily make socket blocking for large send
            self.get_logger().info('Setting socket to blocking mode...')
            original_blocking = self.client_sock.getblocking()
            self.client_sock.setblocking(True)
            try:
                self.get_logger().info('Sending image data to AVP...')
                self.client_sock.sendall(image_message.encode('utf-8'))
                self.get_logger().info('Sent image to AVP for annotation')
            finally:
                # Restore original blocking state
                self.get_logger().info('Restoring socket blocking state...')
                self.client_sock.setblocking(original_blocking)
            
            # Set up waiting state
            with self.annotation_lock:
                self.annotation_response = None
                self.waiting_for_annotation = True
                self.annotation_start_time = time.time()
            
            self.get_logger().info('Waiting for annotation response (non-blocking)...')
                
        except Exception as e:
            self.get_logger().error(f'Annotation command failed: {e}')
            raise


    def process_annotations_and_continue(self):
        """Process received annotations and continue with original pipeline"""
        try:
            # Convert normalized coordinates to pixel coordinates
            # TODO: Confirm this is the correct image source
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
            
            # Now trigger the original ParaSight flow
            self.start_pub.publish(Empty())
            self.get_logger().info('Started FSM /trigger_host_ui with AVP annotations')
            
        except Exception as e:
            self.get_logger().error(f'Failed to process annotations: {e}')
            raise

    def handle_command(self, command: str):
        if command == "annotate":
            self.handle_annotate_command()
        elif command == "restart":
            self.stop_pub.publish(Empty())
            self.get_logger().info('Stopped FSM /hard_reset_host')
        elif command == "KILLALL":
            self.handle_emergency_stop()
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
            # try:
            #     index = int(command.split("_")[-1])
            #     self.call_select_pose_service(index)
            # except ValueError:
            #     self.get_logger().warn(f'Invalid pose index in command: "{command}"')

        else:
            self.get_logger().warn(f'Unknown command: "{command}"')

    def call_select_pose_service(self, index: int):
        if not self.select_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service /select_pose not available')
            return

        request = SelectPose.Request()
        request.index = index

        future = self.select_pose_client.call_async(request)
        future.add_done_callback(self.handle_service_response)
        self.get_logger().info(f"Drilling bone index: {index}")

    def handle_service_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Service responded: {response}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def handle_emergency_stop(self):
        """Handle emergency stop command - kill all Docker containers and processes"""
        self.get_logger().error('EMERGENCY STOP ACTIVATED - Killing all containers and processes')
        
        try:
            # Kill all running Docker containers
            subprocess.run(['docker', 'kill', '$(docker ps -q)'], shell=True, check=False)
            self.get_logger().info('Killed all running Docker containers')
            
            # Kill any remaining ROS2 processes
            subprocess.run(['pkill', '-f', 'ros2'], check=False)
            subprocess.run(['pkill', '-f', 'rclpy'], check=False)
            
            self.get_logger().info('Emergency stop completed - all processes terminated')
            
        except Exception as e:
            self.get_logger().error(f'Error during emergency stop: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TcpServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.client_sock:
            node.client_sock.close()
        node.server_sock.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



# # ros2_tcp_server_node.py
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# import socket

# class TcpServerNode(Node):
#     def __init__(self):
#         super().__init__('tcp_server_node')
#         # ROS 2 publisher to /robot_control topic
#         self.publisher = self.create_publisher(String, 'robot_control', 10)
#         self.get_logger().info('ROS2 TCP Server Node started')

#         self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#         self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
#         self.server_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
#         self.server_sock.bind(('0.0.0.0', 5000))   # listen on port 5000
#         self.server_sock.listen(1)
#         self.server_sock.setblocking(False)       # make socket non-blocking

#         self.client_sock = None
#         self.client_addr = None

#         # Use a timer to periodically check for new connections or data
#         self.timer = self.create_timer(0.1, self.poll_socket)

#     def poll_socket(self):
#         # If no client is connected, try to accept a new connection
#         if self.client_sock is None:
#             try:
#                 client, addr = self.server_sock.accept()
#             except BlockingIOError:
#                 return  # no incoming connection yet
#             self.client_sock = client
#             self.client_addr = addr
#             self.client_sock.setblocking(False)
#             self.get_logger().info(f'Accepted TCP connection from {addr}')
#             return

#         # If a client is connected, try to read data
#         try:
#             data = self.client_sock.recv(1024)
#         except BlockingIOError:
#             return  # no data received this cycle

#         if not data:
#             # Client disconnected (empty data)
#             self.get_logger().info('Client disconnected')
#             self.client_sock.close()
#             self.client_sock = None
#             self.client_addr = None
#         else:
#             # Decode and strip the message
#             message = data.decode('utf-8').strip()
#             if message:
#                 self.get_logger().info(f'Received command: "{message}"')
#                 # Publish the command to the ROS2 topic
#                 ros_msg = String()
#                 ros_msg.data = message
#                 self.publisher.publish(ros_msg)
#                 self.get_logger().info(f'Published to /robot_control: "{message}"')
#                 # Send acknowledgment back to client
#                 try:
#                     self.client_sock.sendall(b'acknowledged\n')
#                 except Exception as e:
#                     self.get_logger().error(f'Failed to send acknowledgment: {e}')

# def main(args=None):
#     rclpy.init(args=args)
#     node = TcpServerNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         # Clean up
#         if node.client_sock:
#             node.client_sock.close()
#         node.server_sock.close()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

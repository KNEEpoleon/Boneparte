#!/usr/bin/env python3
"""
AVP TCP Server Node

Streams drill pose data to Apple Vision Pro via TCP.

Subscribes to: /aruco_drill_poses (PoseArray in aruco_marker frame)
TCP Server: Port 5001
Protocol: POSES|x1,y1,z1,qx1,qy1,qz1,qw1|x2,y2,z2,qx2,qy2,qz2,qw2|...\n
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
import socket
import threading


class AVPTcpServer(Node):
    def __init__(self):
        super().__init__('avp_tcp_server')
        self.get_logger().info('AVP TCP Server Node started')
        
        # Latest drill poses
        self.latest_poses = None
        self.poses_lock = threading.Lock()
        
        # Subscribe to transformed drill poses
        self.subscriber = self.create_subscription(
            PoseArray,
            '/aruco_drill_poses',
            self.poses_callback,
            10
        )
        
        # TCP Server setup
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.server_sock.bind(('0.0.0.0', 5001))  # Port 5001 for ArucoTransform app
        self.server_sock.listen(1)
        self.server_sock.setblocking(False)
        
        self.client_sock = None
        self.client_addr = None
        
        # Timer to poll socket and send data
        self.timer = self.create_timer(0.033, self.poll_socket)  # ~30 Hz
        
        self.get_logger().info('TCP server listening on port 5001 for AVP connections')
    
    def poses_callback(self, msg: PoseArray):
        """Store latest drill poses"""
        with self.poses_lock:
            self.latest_poses = msg
        
        self.get_logger().debug(f'Received {len(msg.poses)} drill poses in aruco_marker frame')
    
    def format_poses_message(self, poses: PoseArray) -> str:
        """
        Format poses as: POSES|x,y,z,qx,qy,qz,qw|x,y,z,qx,qy,qz,qw|...
        """
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
    
    def poll_socket(self):
        """Poll for connections and send drill pose data"""
        # Accept new connections if no client connected
        if self.client_sock is None:
            try:
                client, addr = self.server_sock.accept()
            except BlockingIOError:
                return  # No incoming connection
            
            self.client_sock = client
            self.client_addr = addr
            self.client_sock.setblocking(False)
            self.get_logger().info(f'AVP client connected from {addr}')
            return
        
        # Send latest drill poses if available
        with self.poses_lock:
            if self.latest_poses is not None and len(self.latest_poses.poses) > 0:
                try:
                    message = self.format_poses_message(self.latest_poses)
                    self.client_sock.sendall(message.encode('utf-8'))
                    
                    # Log occasionally (not every frame to avoid spam)
                    if self.get_clock().now().nanoseconds % 1000000000 < 33000000:  # ~once per second
                        self.get_logger().info(
                            f'Sent {len(self.latest_poses.poses)} drill poses to AVP'
                        )
                
                except (BrokenPipeError, ConnectionResetError, OSError) as e:
                    self.get_logger().warn(f'Client disconnected: {e}')
                    self.client_sock.close()
                    self.client_sock = None
                    self.client_addr = None
        
        # Check for client messages (heartbeat, etc.)
        if self.client_sock is not None:
            try:
                data = self.client_sock.recv(1024)
                if not data:
                    # Client disconnected gracefully
                    self.get_logger().info('Client disconnected')
                    self.client_sock.close()
                    self.client_sock = None
                    self.client_addr = None
                else:
                    # Echo acknowledgment (optional)
                    message = data.decode('utf-8').strip()
                    self.get_logger().debug(f'Received from AVP: {message}')
            except BlockingIOError:
                pass  # No data available
            except Exception as e:
                self.get_logger().error(f'Error receiving data: {e}')
                self.client_sock.close()
                self.client_sock = None
                self.client_addr = None
    
    def __del__(self):
        """Cleanup on node destruction"""
        if self.client_sock:
            self.client_sock.close()
        self.server_sock.close()


def main(args=None):
    rclpy.init(args=args)
    node = AVPTcpServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


#!/usr/bin/env python3
# ros2_tcp_server_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from surgical_robot_planner.srv import SelectPose
import socket

class TcpServerNode(Node):
    def __init__(self):
        super().__init__('tcp_server_node')
        self.get_logger().info('ROS2 TCP Server Node started')

        # One-shot publisher
        self.start_pub = self.create_publisher(Empty, '/trigger_host_ui', 10)
        self.stop_pub = self.create_publisher(Empty, '/hard_reset_host', 10)

        # Service client
        self.select_pose_client = self.create_client(SelectPose, '/select_pose')

        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.server_sock.bind(('0.0.0.0', 5000))
        self.server_sock.listen(1)
        self.server_sock.setblocking(False)

        self.client_sock = None
        self.client_addr = None

        self.timer = self.create_timer(0.1, self.poll_socket)

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

        try:
            data = self.client_sock.recv(1024)
        except BlockingIOError:
            return

        if not data:
            self.get_logger().info('Client disconnected')
            self.client_sock.close()
            self.client_sock = None
            self.client_addr = None
        else:
            message = data.decode('utf-8').strip()
            if message:
                self.get_logger().info(f'Received command: "{message}"')
                self.handle_command(message)

                try:
                    self.client_sock.sendall(b'acknowledged\n')
                except Exception as e:
                    self.get_logger().error(f'Failed to send acknowledgment: {e}')

    def handle_command(self, command: str):
        if command == "annotate":
            self.start_pub.publish(Empty())
            self.get_logger().info('Started FSM /trigger_host_ui')
        elif command == "restart":
            self.stop_pub.publish(Empty())
            self.get_logger().info('Stopped FSM /hard_reset_host')

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

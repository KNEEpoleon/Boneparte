#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient

class MoveItActionClient(Node):
    def __init__(self):
        super().__init__('moveit_action_client')
        self.client = ActionClient(self, FollowJointTrajectory, 'lbr/joint_trajectory_controller/follow_joint_trajectory')
        self.client.wait_for_server()
        self.send_goal()

    def send_goal(self):
        goal_msg = FollowJointTrajectory.Goal()
        
        goal_msg.trajectory.joint_names = [
            'lbr_A1', 'lbr_A2', 'lbr_A3', 'lbr_A4', 'lbr_A5', 'lbr_A6', 'lbr_A7'
        ]

        # target joint positions
        point = JointTrajectoryPoint()
        point.positions = [0.0, -0.6981, 0.0, 1.0472, 0.0, -1.5708, 0.0]
        # [0.7505, 1.0647, -0.5236, -0.9599, 0.0, 1.6581, 0.0]
        point.time_from_start.sec = 5  # Motion should complete in 5 seconds

        goal_msg.trajectory.points.append(point)

        self.get_logger().info("Sending trajectory goal...")
        self.future = self.client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return
        self.get_logger().info("Goal accepted! Waiting for execution...")
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code == 0:
            self.get_logger().info("Trajectory execution successful!")
        else:
            self.get_logger().error(f"Execution failed with error code: {result.error_code}")

def main(args=None):
    rclpy.init(args=args)
    node = MoveItActionClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

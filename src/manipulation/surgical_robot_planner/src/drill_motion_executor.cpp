#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class PoseSubscriberNode : public rclcpp::Node {
private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr drill_command_publisher_;
  std::string robot_name_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;

public:
  static std::shared_ptr<PoseSubscriberNode> create() {
    return std::make_shared<PoseSubscriberNode>();
  }

  PoseSubscriberNode() : Node("move_to_pose_node") {
    this->declare_parameter("robot_name", "lbr");
    robot_name_ = this->get_parameter("robot_name").as_string();
  }

  void initialize() {
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        std::static_pointer_cast<rclcpp::Node>(shared_from_this()),
        moveit::planning_interface::MoveGroupInterface::Options(
            "arm", "robot_description", robot_name_));

    subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/surgical_drill_pose", 10,
        std::bind(&PoseSubscriberNode::pose_callback, this, std::placeholders::_1));

    drill_command_publisher_ = this->create_publisher<std_msgs::msg::String>("/drill_commands", 10);
  }

  void pose_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    if (msg->poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty pose array.");
      return;
    }

    geometry_msgs::msg::Pose target_pose = msg->poses[0];
    geometry_msgs::msg::Pose above_pose = target_pose;

    tf2::Quaternion q(
        target_pose.orientation.x,
        target_pose.orientation.y,
        target_pose.orientation.z,
        target_pose.orientation.w);
    tf2::Matrix3x3 rot(q);
    tf2::Vector3 offset = rot * tf2::Vector3(0, 0, -0.1);

    above_pose.position.x += offset.x();
    above_pose.position.y += offset.y();
    above_pose.position.z += offset.z();

    move_group_interface_->setStartStateToCurrentState();
    move_group_interface_->setPoseTarget(above_pose, robot_name_ + "_link_ee");
    move_group_interface_->setWorkspace(-0.9, -0.55, 1.0, 0.1, 0.55, 1000.0);

    moveit::planning_interface::MoveGroupInterface::Plan plan_above;
    if (move_group_interface_->plan(plan_above) == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Moving to pre-drill pose...");
      move_group_interface_->execute(plan_above);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to plan to pre-drill pose.");
      return;
    }

    std::vector<geometry_msgs::msg::Pose> waypoints{above_pose, target_pose};
    moveit_msgs::msg::RobotTrajectory trajectory;
    move_group_interface_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    auto start_msg = std_msgs::msg::String();
    start_msg.data = "start";
    drill_command_publisher_->publish(start_msg);
    RCLCPP_INFO(this->get_logger(), "Published 'start' to /drill_commands.");

    move_group_interface_->setMaxVelocityScalingFactor(0.02);
    move_group_interface_->setMaxAccelerationScalingFactor(0.02);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    RCLCPP_INFO(this->get_logger(), "Drilling motion executing...");
    move_group_interface_->execute(plan);

    auto stop_msg = std_msgs::msg::String();
    stop_msg.data = "stop";
    drill_command_publisher_->publish(stop_msg);
    RCLCPP_INFO(this->get_logger(), "Published 'stop' to /drill_commands.");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = PoseSubscriberNode::create();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

#include "geometry_msgs/msg/pose_array.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"

class PoseSubscriberNode : public rclcpp::Node {
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
  }

private:
  void pose_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    if (msg->poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty pose array.");
      return;
    }

    geometry_msgs::msg::Pose target_pose = msg->poses[0];
    move_group_interface_->setPoseTarget(target_pose, robot_name_ + "_link_ee");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto success = move_group_interface_->plan(plan);

    if (success == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Executing motion to first pose...");
      move_group_interface_->execute(plan);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to plan to the target pose.");
    }
  }

  std::string robot_name_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = PoseSubscriberNode::create();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
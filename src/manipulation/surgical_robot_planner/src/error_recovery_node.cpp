#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/string.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class ErrorRecoveryNode : public rclcpp::Node {
private:
  std::string robot_name_;
  std::string camera_frame_;
  std::string base_frame_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr direction_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr error_publisher_;

public:
  ErrorRecoveryNode() : Node("error_recovery_node") {
    this->declare_parameter("robot_name", "lbr");
    this->declare_parameter("camera_frame", "camera_frame");
    this->declare_parameter("base_frame", "lbr_link_0");
    robot_name_ = this->get_parameter("robot_name").as_string();
    camera_frame_ = this->get_parameter("camera_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
  }

public:
  void initialize() {
    // Initialize TF buffer and listener (for error recovery)
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize MoveGroupInterface
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        std::static_pointer_cast<rclcpp::Node>(shared_from_this()),
        moveit::planning_interface::MoveGroupInterface::Options("arm", "robot_description", robot_name_));
    move_group_interface_->setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group_interface_->setPlanningTime(5.0);

    // Subscribe to string commands (bring_home, go_away)
    command_subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/manipulator_command", 10,
        std::bind(&ErrorRecoveryNode::command_callback, this, std::placeholders::_1));
    
    // Subscribe to error recovery direction from perception
    direction_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        "/error_recovery_direction", 10,
        std::bind(&ErrorRecoveryNode::direction_callback, this, std::placeholders::_1));
    
    // Publisher for error recovery status
    error_publisher_ = this->create_publisher<std_msgs::msg::String>("/error_recovery_status", 10);
    
    RCLCPP_INFO(this->get_logger(), "ErrorRecoveryNode initialized for robot: %s", robot_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Camera frame: %s, Base frame: %s", camera_frame_.c_str(), base_frame_.c_str());
  }

private:

  void command_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::string command = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received command: %s", command.c_str());

    if (command == "bring_home") {
      move_to_home();
    } else if (command == "go_away") {
      move_to_away();
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown command: %s. Valid commands are 'bring_home' and 'go_away'.", command.c_str());
    }
  }

  void direction_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received error recovery direction: [%.4f, %.4f, %.4f]", 
                msg->x, msg->y, msg->z);
    
    try {
      // Get current end-effector pose in base frame
      geometry_msgs::msg::PoseStamped current_ee_pose = move_group_interface_->getCurrentPose(robot_name_ + "_link_ee");
      
      // Get transform from camera frame to base frame
      geometry_msgs::msg::TransformStamped camera_to_base;
      try {
        camera_to_base = tf_buffer_->lookupTransform(
            base_frame_, camera_frame_, 
            rclcpp::Time(0), 
            rclcpp::Duration::from_seconds(1.0));
      } catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get transform from %s to %s: %s", 
                     camera_frame_.c_str(), base_frame_.c_str(), ex.what());
        publish_error("TF transform lookup failed");
        return;
      }

      // Convert offset vector from camera frame to base frame
      tf2::Vector3 offset_camera(msg->x, msg->y, msg->z);
      
      // Extract rotation from transform
      tf2::Quaternion q(
          camera_to_base.transform.rotation.x,
          camera_to_base.transform.rotation.y,
          camera_to_base.transform.rotation.z,
          camera_to_base.transform.rotation.w);
      tf2::Matrix3x3 rot(q);
      
      // Transform vector to base frame
      tf2::Vector3 offset_base = rot * offset_camera;
      
      // Calculate new position: apply x,y offset, keep z constant
      geometry_msgs::msg::Pose target_pose = current_ee_pose.pose;
      target_pose.position.x += offset_base.x();
      target_pose.position.y += offset_base.y();
      // Keep z constant (don't modify target_pose.position.z)
      
      RCLCPP_INFO(this->get_logger(), "Current EE pose: [%.4f, %.4f, %.4f]", 
                  current_ee_pose.pose.position.x, 
                  current_ee_pose.pose.position.y, 
                  current_ee_pose.pose.position.z);
      RCLCPP_INFO(this->get_logger(), "Offset in base frame: [%.4f, %.4f, %.4f]", 
                  offset_base.x(), offset_base.y(), offset_base.z());
      RCLCPP_INFO(this->get_logger(), "Target EE pose: [%.4f, %.4f, %.4f]", 
                  target_pose.position.x, target_pose.position.y, target_pose.position.z);
      
      // Plan and execute movement
      move_group_interface_->setStartStateToCurrentState();
      move_group_interface_->setPoseTarget(target_pose, robot_name_ + "_link_ee");
      move_group_interface_->setPlannerId("LIN");
      move_group_interface_->setMaxVelocityScalingFactor(0.5);

      moveit::planning_interface::MoveGroupInterface::Plan recovery_plan;
      auto plan_result = move_group_interface_->plan(recovery_plan);
      
      if (plan_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Planning successful. Executing error recovery movement...");
        auto execution_result = move_group_interface_->execute(recovery_plan);
        
        if (execution_result == moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_INFO(this->get_logger(), "Successfully executed error recovery movement.");
          publish_error("success");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to execute error recovery movement with error code: %d", 
                       execution_result.val);
          publish_error("execution_failed");
        }
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to plan error recovery movement with error code: %d", 
                     plan_result.val);
        publish_error("planning_failed");
      }
      
    } catch (const std::exception &ex) {
      RCLCPP_ERROR(this->get_logger(), "Exception in error recovery: %s", ex.what());
      publish_error("exception");
    }
  }

  void move_to_home() {
    RCLCPP_INFO(this->get_logger(), "Moving manipulator to home pose (away -> home)...");
    
    move_group_interface_->setStartStateToCurrentState();
    move_group_interface_->setNamedTarget("Boneparte_home");
    move_group_interface_->setPlannerId("PTP");
    move_group_interface_->setMaxVelocityScalingFactor(1.0);

    moveit::planning_interface::MoveGroupInterface::Plan home_plan;
    auto plan_result = move_group_interface_->plan(home_plan);
    
    if (plan_result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Planning successful. Executing movement to home pose...");
      auto execution_result = move_group_interface_->execute(home_plan);
      
      if (execution_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Successfully moved to home pose.");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to execute movement to home pose with error code: %d", execution_result.val);
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to plan movement to home pose with error code: %d", plan_result.val);
    }
  }

  void move_to_away() {
    RCLCPP_INFO(this->get_logger(), "Moving manipulator to away pose (home -> away)...");
    
    move_group_interface_->setStartStateToCurrentState();
    move_group_interface_->setNamedTarget("Boneparte_away");
    move_group_interface_->setPlannerId("PTP");
    move_group_interface_->setMaxVelocityScalingFactor(1.0);

    moveit::planning_interface::MoveGroupInterface::Plan away_plan;
    auto plan_result = move_group_interface_->plan(away_plan);
    
    if (plan_result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Planning successful. Executing movement to away pose...");
      auto execution_result = move_group_interface_->execute(away_plan);
      
      if (execution_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Successfully moved to away pose.");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to execute movement to away pose with error code: %d", execution_result.val);
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to plan movement to away pose with error code: %d", plan_result.val);
    }
  }

  void publish_error(const std::string& error_msg) {
    auto error_status = std_msgs::msg::String();
    error_status.data = error_msg;
    error_publisher_->publish(error_status);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ErrorRecoveryNode>();
  node->initialize();  // Initialize after shared_ptr is created
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


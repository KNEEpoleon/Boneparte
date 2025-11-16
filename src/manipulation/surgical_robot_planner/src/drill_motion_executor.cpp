#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/collision_object.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "surgical_robot_planner/srv/select_pose.hpp"

class PoseSubscriberNode : public rclcpp::Node {
private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr drill_command_publisher_;
  std::string robot_name_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_; 
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
  rclcpp::Service<surgical_robot_planner::srv::SelectPose>::SharedPtr select_pose_service_;
  std::vector<geometry_msgs::msg::Pose> stored_poses_;
  int pin_counter = 0;

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
        moveit::planning_interface::MoveGroupInterface::Options("arm", "robot_description", robot_name_));
    move_group_interface_->setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group_interface_ -> setPlanningTime(5.0);

    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>(robot_name_);

    subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/surgical_drill_pose", 10,
        std::bind(&PoseSubscriberNode::pose_callback, this, std::placeholders::_1));
    drill_command_publisher_ = this->create_publisher<std_msgs::msg::String>("/drill_commands", 10);
    select_pose_service_ = this->create_service<surgical_robot_planner::srv::SelectPose>("/select_pose",
      std::bind(&PoseSubscriberNode::select_pose_callback, this, std::placeholders::_1, std::placeholders::_2));
  }
  void pose_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    if (msg->poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty pose array.");
      return;
    }
    stored_poses_ = msg->poses;
    RCLCPP_INFO(this->get_logger(), "Stored %zu poses. Ready for user selection via service.", stored_poses_.size());
  }
  
  void select_pose_callback(const std::shared_ptr<surgical_robot_planner::srv::SelectPose::Request> request,std::shared_ptr<surgical_robot_planner::srv::SelectPose::Response> response) {
    int index = request->index;
    if (index < 0 || static_cast<size_t>(index) >= stored_poses_.size()) {
      response->success = false;
      response->message = "Invalid pose index selected.";
      RCLCPP_ERROR(this->get_logger(), "User selected invalid pose index %d.", index);
      return;
    }
    RCLCPP_INFO(this->get_logger(), "User selected pose index %d.", index);
    drill_at_pose(stored_poses_[index]);
    response->success = true;
    response->message = "Calling drill at selected pose.";
  }

  void add_drilled_pin_as_obstacle(const geometry_msgs::msg::Pose& pin_pose) {
    pin_counter++;
    
    // Create a unique ID for each pin
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = robot_name_ + "_link_0"; 
    
    collision_object.id = "drilled_pin_" + std::to_string(pin_counter);

    shape_msgs::msg::SolidPrimitive primitive;
    // Declare a cylinder obstacle for the pin
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = 0.10;   // surgical pin length 12cm
    // primitive.dimensions[0] = 0.16;   // surgical pin length 16cm - svd encore
    primitive.dimensions[1] = 0.004;    // accuracy cylinder radius 8mm

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pin_pose);
    collision_object.operation = collision_object.ADD;

    planning_scene_interface_->applyCollisionObjects({collision_object}); 

    RCLCPP_INFO(this->get_logger(), "Added drilled pin as obstacle.");
  }
  
  void drill_at_pose(const geometry_msgs::msg::Pose& target_pose) {
    geometry_msgs::msg::Pose above_pose = target_pose;
    geometry_msgs::msg::Pose final_pose= target_pose;

    tf2::Quaternion q(
        target_pose.orientation.x,
        target_pose.orientation.y,
        target_pose.orientation.z,
        target_pose.orientation.w);
    tf2::Matrix3x3 rot(q);

    // A point 12cm above drill site along drill axis
    tf2::Vector3 offset = rot * tf2::Vector3(0, 0, -0.16);             // 16cm above the drill pose
    tf2::Vector3 offset_2 = rot * tf2::Vector3(0, 0, -0.047);

    above_pose.position.x += offset.x();
    above_pose.position.y += offset.y();
    above_pose.position.z += offset.z();

    final_pose.position.x += offset_2.x();
    final_pose.position.y += offset_2.y();
    final_pose.position.z += offset_2.z();

    // Move to pre-drill position
    move_group_interface_->setStartStateToCurrentState();
    move_group_interface_->setPoseTarget(above_pose, robot_name_ + "_link_ee");
    move_group_interface_->setPlannerId("LIN");
    move_group_interface_->setMaxVelocityScalingFactor(1);

    moveit::planning_interface::MoveGroupInterface::Plan plan_above;
    auto pre_drill_plan = move_group_interface_->plan(plan_above);
    if (pre_drill_plan == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Moving to pre-drill pose...");
      auto execution_result = move_group_interface_->execute(plan_above);
      
      if (execution_result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Failed to execute pre-drill movement with error code: %d", execution_result.val);
        return;
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to plan to pre-drill pose with error code: %d", pre_drill_plan.val);
      return;
    }

    // Move to drill position 
    move_group_interface_->setStartStateToCurrentState();
    move_group_interface_->setPoseTarget(final_pose, robot_name_ + "_link_ee");
    move_group_interface_->setPlannerId("LIN");
    // move_group_interface_->setMaxVelocityScalingFactor(0.00485);  //0.001->1mm/sec
    move_group_interface_->setMaxVelocityScalingFactor(0.006);      //0.001->1mm/sec

    moveit::planning_interface::MoveGroupInterface::Plan plan_drill;
    auto drilling_plan = move_group_interface_->plan(plan_drill);
    if (drilling_plan == moveit::core::MoveItErrorCode::SUCCESS) {    
      // Start drill only after successful motion planning
      auto start_msg = std_msgs::msg::String();
      start_msg.data = "start";
      drill_command_publisher_->publish(start_msg);
      RCLCPP_INFO(this->get_logger(), "Published 'start' to /drill_commands.");
      
      // Execute the drilling motion
      RCLCPP_INFO(this->get_logger(), "Drilling motion executing...");
      auto execution_result = move_group_interface_->execute(plan_drill);
      
      // Stop drill after motion completes or fails
      auto stop_msg = std_msgs::msg::String();
      stop_msg.data = "stop";
      drill_command_publisher_->publish(stop_msg);
      RCLCPP_INFO(this->get_logger(), "Published 'stop' to /drill_commands.");
      
      if (execution_result == moveit::core::MoveItErrorCode::SUCCESS) {
        // Plan and execute return to pre-drill position
        return_to_pre_drill_position(above_pose);
        add_drilled_pin_as_obstacle(target_pose);
        return_to_home_pose();
      } else {
        RCLCPP_ERROR(this->get_logger(), "Drilling motion execution failed with error code: %d", execution_result.val);
        return_to_home_pose();
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Drilling motion planning failed with error code: %d", drilling_plan.val);
      return_to_home_pose();
    }
  }

  void return_to_pre_drill_position(const geometry_msgs::msg::Pose& above_pose) {
    RCLCPP_INFO(this->get_logger(), "Planning return to pre-drill position...");
    
    move_group_interface_->setStartStateToCurrentState();
    move_group_interface_->setPoseTarget(above_pose, robot_name_ + "_link_ee");
    move_group_interface_->setPlannerId("LIN"); 
    
    // Reset to reasonable speed for retraction
    move_group_interface_->setMaxVelocityScalingFactor(0.5);  
    
    moveit::planning_interface::MoveGroupInterface::Plan retract_plan;
    auto plan_result = move_group_interface_->plan(retract_plan);
    if (plan_result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Returning to pre-drill position...");
      auto execution_result = move_group_interface_->execute(retract_plan);
      
      if (execution_result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Failed to execute return to pre-drill position with error code: %d", execution_result.val);
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to plan return to pre-drill position with error code: %d", plan_result.val);
    }
  }

  void return_to_home_pose() {
    RCLCPP_INFO(this->get_logger(), "Attempting to return to home pose due to error...");
    
    move_group_interface_->setStartStateToCurrentState();
    move_group_interface_->setNamedTarget("Boneparte_home");  // target from SRDF
    move_group_interface_->setPlannerId("PTP");
    move_group_interface_->setMaxVelocityScalingFactor(1);

        
    moveit::planning_interface::MoveGroupInterface::Plan home_plan;
    auto plan_result = move_group_interface_->plan(home_plan);
    if (plan_result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Moving to home pose for recovery...");
      move_group_interface_->execute(home_plan);
    } else {
      RCLCPP_ERROR(this->get_logger(), "CRITICAL: Failed to plan return to home pose with error code: %d", plan_result.val);
      RCLCPP_ERROR(this->get_logger(), "Manual recovery may be required.");
    }
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

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
#include <vector>
#include <algorithm>

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
  
  // New members for dynamic obstacle management
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr registration_subscription_;
  std::string current_registration_state_ = "idle";
  std::vector<std::string> drilled_pin_ids_;  // Track actual collision object IDs
  std::vector<int> pin_pose_indices_;         // Track which pose index each pin corresponds to
  bool drilling_blocked_ = false;  // Block drilling during reregistration

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
    
    // Subscribe to registration state
    registration_subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/registration", 10,
        std::bind(&PoseSubscriberNode::registration_callback, this, std::placeholders::_1));
  }
  void pose_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    if (msg->poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty pose array.");
      return;
    }
    stored_poses_ = msg->poses;
    RCLCPP_INFO(this->get_logger(), "Stored %zu poses. Frame: %s", stored_poses_.size(), msg->header.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "First pose: [%.3f, %.3f, %.3f]", 
                stored_poses_[0].position.x, stored_poses_[0].position.y, stored_poses_[0].position.z);
  }
  
  void select_pose_callback(const std::shared_ptr<surgical_robot_planner::srv::SelectPose::Request> request,std::shared_ptr<surgical_robot_planner::srv::SelectPose::Response> response) {
    if (drilling_blocked_) {
      response->success = false;
      response->message = "Drilling blocked during reregistration. Wait for 'complete' message.";
      RCLCPP_ERROR(this->get_logger(), "Drilling blocked - reregistration in progress!");
      return;
    }
    
    int index = request->index;
    if (index < 0 || static_cast<size_t>(index) >= stored_poses_.size()) {
      response->success = false;
      response->message = "Invalid pose index selected.";
      RCLCPP_ERROR(this->get_logger(), "User selected invalid pose index %d.", index);
      return;
    }
    RCLCPP_INFO(this->get_logger(), "User selected pose index %d.", index);
    drill_at_pose(stored_poses_[index], index);
    response->success = true;
    response->message = "Calling drill at selected pose.";
  }

  void add_drilled_pin_as_obstacle(const geometry_msgs::msg::Pose& pin_pose, int pose_index) {
    pin_counter++;
    std::string pin_id = "drilled_pin_" + std::to_string(pin_counter);
    
    // Track this pin
    drilled_pin_ids_.push_back(pin_id);
    pin_pose_indices_.push_back(pose_index);
    
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = robot_name_ + "_link_0"; 
    collision_object.id = pin_id;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = 0.12;
    primitive.dimensions[1] = 0.008;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pin_pose);
    collision_object.operation = collision_object.ADD;

    planning_scene_interface_->applyCollisionObjects({collision_object}); 

    RCLCPP_INFO(this->get_logger(), "Added drilled pin %s as obstacle at pose index %d.", 
                pin_id.c_str(), pose_index);
  }

  void drill_at_pose(const geometry_msgs::msg::Pose& target_pose, int pose_index) {
    geometry_msgs::msg::Pose above_pose = target_pose;
    geometry_msgs::msg::Pose final_pose= target_pose;

    tf2::Quaternion q(
        target_pose.orientation.x,
        target_pose.orientation.y,
        target_pose.orientation.z,
        target_pose.orientation.w);
    tf2::Matrix3x3 rot(q);

    // A point 12cm above drill site along drill axis
    tf2::Vector3 offset = rot * tf2::Vector3(0, 0, -0.15);
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
        add_drilled_pin_as_obstacle(target_pose, pose_index);
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

  void registration_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::string new_state = msg->data;
    
    if (new_state == "reregistering") {
      RCLCPP_INFO(this->get_logger(), "Reregistration started. Removing all drilled pin obstacles...");
      drilling_blocked_ = true;
      remove_all_drilled_pin_obstacles();
    } else if (new_state == "complete") {
      RCLCPP_INFO(this->get_logger(), "Registration complete. Re-adding pin obstacles at new poses...");
      readd_drilled_pin_obstacles();
      drilling_blocked_ = false;
      RCLCPP_INFO(this->get_logger(), "Drilling unblocked. System ready.");
    }
    
    current_registration_state_ = new_state;
    RCLCPP_INFO(this->get_logger(), "Registration state: %s", new_state.c_str());
  }

  void remove_all_drilled_pin_obstacles() {
    if (drilled_pin_ids_.empty()) {
      RCLCPP_INFO(this->get_logger(), "No drilled pins to remove.");
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Removing %zu drilled pin obstacles...", drilled_pin_ids_.size());
    log_drilled_pins_status();
    
    for (size_t i = 0; i < drilled_pin_ids_.size(); i++) {
      moveit_msgs::msg::CollisionObject remove_obj;
      remove_obj.id = drilled_pin_ids_[i];
      remove_obj.header.frame_id = robot_name_ + "_link_0";
      remove_obj.operation = remove_obj.REMOVE;
      
      planning_scene_interface_->applyCollisionObjects({remove_obj});
      RCLCPP_INFO(this->get_logger(), "Removed obstacle: %s", drilled_pin_ids_[i].c_str());
    }
    
    RCLCPP_INFO(this->get_logger(), "All drilled pin obstacles removed. Waiting for new poses...");
  }
  
  void readd_drilled_pin_obstacles() {
    if (drilled_pin_ids_.empty()) {
      RCLCPP_INFO(this->get_logger(), "No drilled pins to re-add.");
      return;
    }
    
    if (stored_poses_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No poses available! Cannot re-add obstacles.");
      return;
    }
    
    // Check size mismatch
    int max_pose_idx = *std::max_element(pin_pose_indices_.begin(), pin_pose_indices_.end());
    if (max_pose_idx >= static_cast<int>(stored_poses_.size())) {
      RCLCPP_ERROR(this->get_logger(), 
                   "ERROR: Pose array size mismatch! Have %zu poses but need at least %d poses for drilled pins.",
                   stored_poses_.size(), max_pose_idx + 1);
      RCLCPP_ERROR(this->get_logger(), "Cannot re-add obstacles. System in inconsistent state!");
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Re-adding %zu drilled pin obstacles at new poses...", drilled_pin_ids_.size());
    log_drilled_pins_status();
    
    for (size_t i = 0; i < drilled_pin_ids_.size(); i++) {
      int pose_idx = pin_pose_indices_[i];
      
      if (pose_idx >= 0 && pose_idx < static_cast<int>(stored_poses_.size())) {
        const auto& new_pose = stored_poses_[pose_idx];
        
        RCLCPP_INFO(this->get_logger(), "DEBUG: Re-adding pin %s at pose_index=%d", 
                   drilled_pin_ids_[i].c_str(), pose_idx);
        RCLCPP_INFO(this->get_logger(), "DEBUG: new_pose from stored_poses_[%d] = [%.3f, %.3f, %.3f]",
                   pose_idx, new_pose.position.x, new_pose.position.y, new_pose.position.z);
        RCLCPP_INFO(this->get_logger(), "DEBUG: Orientation: [%.3f, %.3f, %.3f, %.3f]",
                   new_pose.orientation.x, new_pose.orientation.y, new_pose.orientation.z, new_pose.orientation.w);
        
        // Create collision object
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.id = drilled_pin_ids_[i];
        collision_object.header.frame_id = robot_name_ + "_link_0";
        
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.CYLINDER;
        primitive.dimensions.resize(2);
        primitive.dimensions[0] = 0.12;   // surgical pin length 12cm
        primitive.dimensions[1] = 0.008;  // accuracy cylinder radius 8mm
        
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(new_pose);
        collision_object.operation = collision_object.ADD;
        
        planning_scene_interface_->applyCollisionObjects({collision_object});
        RCLCPP_INFO(this->get_logger(), "Re-added pin %s at position: [%.3f, %.3f, %.3f]", 
                   drilled_pin_ids_[i].c_str(), new_pose.position.x, new_pose.position.y, new_pose.position.z);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Pin %s has invalid pose_index %d (poses_size=%zu)", 
                    drilled_pin_ids_[i].c_str(), pose_idx, stored_poses_.size());
      }
    }
    
    RCLCPP_INFO(this->get_logger(), "All drilled pins re-added successfully!");
  }
  void move_drilled_pin_obstacle(const std::string& pin_id, const geometry_msgs::msg::Pose& new_pose) {
    // Create collision object for moving the pin
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.id = pin_id;  // Use the pin_id directly
    collision_object.header.frame_id = robot_name_ + "_link_0";
    collision_object.operation = collision_object.MOVE;
    
    // Create primitive (same dimensions as original pin)
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = 0.12;   // surgical pin length 12cm
    primitive.dimensions[1] = 0.008;  // accuracy cylinder radius 8mm
    
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(new_pose);
    
    // Apply the move operation
    try {
      planning_scene_interface_->applyCollisionObjects({collision_object});
      RCLCPP_INFO(this->get_logger(), "Successfully moved pin %s to new position: [%.3f, %.3f, %.3f]", 
                 pin_id.c_str(), new_pose.position.x, new_pose.position.y, new_pose.position.z);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to move pin %s: %s", pin_id.c_str(), e.what());
    }
  }
  void log_drilled_pins_status() {
    RCLCPP_INFO(this->get_logger(), "=== Drilled Pins Status ===");
    RCLCPP_INFO(this->get_logger(), "Total pins drilled: %zu", drilled_pin_ids_.size());
    RCLCPP_INFO(this->get_logger(), "Available poses: %zu", stored_poses_.size());
    
    for (size_t i = 0; i < drilled_pin_ids_.size(); i++) {
      RCLCPP_INFO(this->get_logger(), "Pin %s: pose_index=%d",
                 drilled_pin_ids_[i].c_str(), pin_pose_indices_[i]);
    }
    RCLCPP_INFO(this->get_logger(), "==========================");
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

// We identified and fixed critical bugs in the surgical robot pin obstacle management system during reregistration.
//  The original code had a redundant `drilled_pins_` boolean vector that was always true and caused indexing 
// confusion. More critically, there was a mismatch in how pin collision objects were tracked and referenced - 
// pins were created with IDs based on `pin_counter` (e.g., "drilled_pin_1", "drilled_pin_2") but the 
// `move_drilled_pin_obstacle` function attempted to reconstruct these IDs using loop indices, which failed when 
// pins were drilled out of order. The function signature also had inconsistencies with some versions expecting 
// int parameters and others expecting string parameters, plus undefined variables like `vector_index` being used 
// instead of actual parameter names. The solution eliminated the `drilled_pins_` vector entirely and introduced 
// `drilled_pin_ids_` to explicitly store the actual collision object ID strings created during drilling. 
// The `move_drilled_pin_obstacle` function signature was standardized to accept `const std::string& pin_id` directly
//  rather than attempting index-based ID reconstruction. This ensures that when reregistration occurs and new poses 
//  arrive on the `/surgical_drill_pose` topic, the system can correctly move each pin obstacle to its corresponding 
//  new pose by directly referencing the stored collision object ID and the stored pose index, maintaining proper 
//  obstacle tracking regardless of drilling order. The refactored code removes all redundant checks, eliminates 
//  compilation errors from type mismatches and undefined variables, and provides robust tracking of which pose index 
//  each drilled pin corresponds to for accurate obstacle updates during reregistration.
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/collision_object.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "rclcpp/rclcpp.hpp"
#include <vector>

class ObstacleManagerNode : public rclcpp::Node {
private:
  std::string robot_name_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  
  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pin_drilled_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_subscription_;
  
  // State
  std::vector<geometry_msgs::msg::Pose> stored_poses_;
  std::vector<std::string> drilled_pin_ids_;
  int pin_counter_ = 0;

public:
  ObstacleManagerNode() : Node("obstacle_manager") {
    this->declare_parameter("robot_name", "lbr");
    robot_name_ = this->get_parameter("robot_name").as_string();
    
    initialize();
  }

private:
  void initialize() {
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>(robot_name_);
    
    // Subscribe to pin_drilled topic
    pin_drilled_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/pin_drilled", 10,
        std::bind(&ObstacleManagerNode::pin_drilled_callback, this, std::placeholders::_1));
    
    // Subscribe to surgical_drill_pose for updated poses
    pose_array_subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/surgical_drill_pose", 10,
        std::bind(&ObstacleManagerNode::pose_array_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "ObstacleManager initialized for robot: %s", robot_name_.c_str());
  }
  
  void pin_drilled_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    add_drilled_pin_as_obstacle(*msg);
  }
  
  void add_drilled_pin_as_obstacle(const geometry_msgs::msg::Pose& pin_pose) {
    pin_counter_++;
    std::string pin_id = "drilled_pin_" + std::to_string(pin_counter_);
    
    // Track this pin (order matters - first drilled = index 0, second = index 1, etc.)
    drilled_pin_ids_.push_back(pin_id);
    
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = robot_name_ + "_link_0"; 
    collision_object.id = pin_id;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = 0.12;   // surgical pin length 12cm
    primitive.dimensions[1] = 0.008;  // accuracy cylinder radius 8mm

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pin_pose);
    collision_object.operation = collision_object.ADD;

    planning_scene_interface_->applyCollisionObjects({collision_object}); 

    RCLCPP_INFO(this->get_logger(), "Added drilled pin %s as obstacle (total pins: %zu).", 
                pin_id.c_str(), drilled_pin_ids_.size());
  }
  
  void pose_array_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    if (msg->poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty pose array.");
      return;
    }
    stored_poses_ = msg->poses;
    RCLCPP_INFO(this->get_logger(), "Stored %zu poses.", stored_poses_.size());
    
    // Update drilled pin obstacles when new poses arrive
    if (!drilled_pin_ids_.empty()) {
      RCLCPP_INFO(this->get_logger(), "New poses received. Automatically updating %zu drilled pin obstacle(s)...", 
                  drilled_pin_ids_.size());
      update_drilled_pin_obstacles();
    }
  }
  
  void update_drilled_pin_obstacles() {
    if (drilled_pin_ids_.empty()) {
      RCLCPP_INFO(this->get_logger(), "No drilled pins to update.");
      return;
    }
    
    if (stored_poses_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No poses available for obstacle update.");
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Updating %zu drilled pin obstacles...", drilled_pin_ids_.size());
    
    for (size_t i = 0; i < drilled_pin_ids_.size(); i++) {
      if (i < stored_poses_.size()) {
        const auto& new_pose = stored_poses_[i];
        
        RCLCPP_INFO(this->get_logger(), "Moving pin %s (index=%zu) to [%.3f, %.3f, %.3f]",
                   drilled_pin_ids_[i].c_str(), i,
                   new_pose.position.x, new_pose.position.y, new_pose.position.z);
        
        move_drilled_pin_obstacle(drilled_pin_ids_[i], new_pose);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Pin %s has invalid index %zu (poses_size=%zu)", 
                    drilled_pin_ids_[i].c_str(), i, stored_poses_.size());
      }
    }
  }
  
  void move_drilled_pin_obstacle(const std::string& pin_id, const geometry_msgs::msg::Pose& new_pose) {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.id = pin_id;
    collision_object.header.frame_id = robot_name_ + "_link_0";
    // Note(Daksh): DONT USE .MOVE, MoveIT hates me for some reason
    collision_object.operation = collision_object.ADD;
    
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
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


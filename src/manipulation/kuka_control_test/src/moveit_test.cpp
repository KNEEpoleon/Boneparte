#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

class MoveItTestNode : public rclcpp::Node
{
public:
    MoveItTestNode()
        : Node("moveit_test_node")
    {
        // Wait for MoveIt to fully start
        move_group_node_ = std::make_shared<rclcpp::Node>(
            "moveit_test_move_group_node");
        
        // Create MoveGroupInterface instance, using "manipulator" as planning group
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            move_group_node_, "manipulator");

        // Set maximum velocity and acceleration scaling factors
        move_group_interface_->setMaxVelocityScalingFactor(0.5);
        move_group_interface_->setMaxAccelerationScalingFactor(0.5);

        // Create a service to set target pose
        target_pose_service_ = this->create_service<geometry_msgs::msg::Pose>(
            "set_target_pose",
            std::bind(&MoveItTestNode::handleSetTargetPose, this,
                      std::placeholders::_1, std::placeholders::_2));

        // Set and execute predefined pose
        moveToPreDefinedPose();

        RCLCPP_INFO(this->get_logger(), "MoveIt test node has started");
    }

private:
    void handleSetTargetPose(
        const std::shared_ptr<geometry_msgs::msg::Pose> request,
        std::shared_ptr<geometry_msgs::msg::Pose> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received new target pose");

        // Set target pose
        geometry_msgs::msg::Pose target_pose = *request;
        move_group_interface_->setPoseTarget(target_pose);

        // Plan the motion
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_interface_->plan(plan) ==
                       moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Planning successful, executing motion");
            move_group_interface_->execute(plan);
            *response = target_pose;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Motion planning failed");
            *response = move_group_interface_->getCurrentPose().pose;
        }
    }

    std::shared_ptr<rclcpp::Node> move_group_node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    rclcpp::Service<geometry_msgs::msg::Pose>::SharedPtr target_pose_service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveItTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


void MoveItTestNode::moveToPreDefinedPose()
{
    geometry_msgs::msg::Pose target_pose;
    
    // Set target position (in meters)
    target_pose.position.x = 0.4;    // 40cm forward
    target_pose.position.y = 0.0;    // Center line
    target_pose.position.z = 0.5;    // 50cm height
    
    // Set target orientation (quaternion)
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 1.0;  // Vertical downward orientation

    RCLCPP_INFO(this->get_logger(), "Moving to predefined pose");
    
    // Set target pose
    move_group_interface_->setPoseTarget(target_pose);

    // Plan the motion
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface_->plan(plan) ==
                   moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
        RCLCPP_INFO(this->get_logger(), "Planning successful, executing motion");
        move_group_interface_->execute(plan);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Motion planning failed");
    }
}
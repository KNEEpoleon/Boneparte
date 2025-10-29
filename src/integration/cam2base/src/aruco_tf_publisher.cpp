#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <memory>

class ArucoTfPublisher : public rclcpp::Node
{
public:
  ArucoTfPublisher() : Node("aruco_tf_publisher")
  {
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Publish the static transform
    publish_static_transform();
    
    RCLCPP_INFO(this->get_logger(), "ArUco static transform published: lbr_link_0 -> aruco_marker");
  }

private:
  void publish_static_transform()
  {
    geometry_msgs::msg::TransformStamped transform;

    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = "lbr_link_0";
    transform.child_frame_id = "aruco_marker";

    // Placeholder transform (to be updated after physical measurement)
    // TODO: Update after printing and mounting ArUco marker on robot base
    // Rough estimate: ArUco marker is 10cm in front (-X direction) of lbr_link_0
    transform.transform.translation.x = -0.10;  // -10 cm in X
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;

    // Identity rotation (ArUco marker aligned with robot base frame)
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);  // No rotation
    
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(transform);
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoTfPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


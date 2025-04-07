#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

rclcpp::TimerBase::SharedPtr timer;

void publish_transform(
    const std::shared_ptr<rclcpp::Node> & node,
    const std::shared_ptr<tf2_ros::TransformBroadcaster> & broadcaster)
{
    geometry_msgs::msg::TransformStamped transform;

    transform.header.stamp = node->get_clock()->now();
    transform.header.frame_id = "lbr_link_7";
    transform.child_frame_id = "camera_frame";

    transform.transform.translation.x = -0.07745074;
    transform.transform.translation.y =  0.00696204;
    transform.transform.translation.z =  0.24017838;

    tf2::Matrix3x3 rot(
        0.99490036,  0.04955656,  0.08784881,
       -0.0508049,   0.99863614,  0.01203032,
       -0.08713282, -0.01643212,  0.99606117
    );

    tf2::Quaternion q;
    rot.getRotation(q);

    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    broadcaster->sendTransform(transform);
}

// Callback function for the timer to publish the transform
void timer_callback()
{
    static std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("hand2eye_tf_publisher");
    static std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster =
        std::make_shared<tf2_ros::TransformBroadcaster>(node);

    publish_transform(node, broadcaster);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("hand2eye_tf_publisher");
    auto broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    // Timer now uses a normal function as callback
    timer = node->create_wall_timer(
        100ms, timer_callback
    );

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

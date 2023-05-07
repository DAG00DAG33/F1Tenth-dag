#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class PathManagerNode : public rclcpp::Node
{
public:
  PathManagerNode() : Node("transform_odom_node"), buffer_(this->get_clock()), listener_(buffer_)
  {
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&PathManagerNode::odom_callback, this, std::placeholders::_1));
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("output_path", 10);
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped input_pose;
    input_pose.header = msg->header;
    input_pose.pose = msg->pose.pose;

    geometry_msgs::msg::PoseStamped transformed_pose;
    try
    {
      buffer_.transform(input_pose, transformed_pose, "map", tf2::durationFromSec(1));
      path_.header = transformed_pose.header;
      path_.poses.push_back(transformed_pose);
      path_publisher_->publish(path_);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
    }
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  nav_msgs::msg::Path path_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathManagerNode>());
  rclcpp::shutdown();
  return 0;
}

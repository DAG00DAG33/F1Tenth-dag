#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>
#include <sstream>
#include <std_msgs/msg/int8.hpp>

class PathManagerNode : public rclcpp::Node
{
public:
  PathManagerNode() : Node("transform_odom_node"), buffer_(this->get_clock()), listener_(buffer_)
  {
    this->declare_parameter<std::string>("odom_topic", "/odom");
    this->declare_parameter<std::string>("output_path_topic", "output_path");
    this->declare_parameter<std::string>("output_pose_topic", "output_pose");
    this->declare_parameter<double>("transform_time_buffer", 1.0);
    this->declare_parameter<double>("path_publish_frequency", 1.0);
    this->declare_parameter<double>("path_sampling_frequency", 1.0);
    this->declare_parameter<std::string>("output_file", "path.csv");
    this->declare_parameter<std::string>("input_file", "input_path.csv");
    this->declare_parameter<bool>("save_path_ena", true);
    this->declare_parameter<bool>("load_path_ena", false);
    this->declare_parameter<bool>("sample_path_ena", true);
    this->declare_parameter<bool>("enable_button_used", false);

    std::string odom_topic = this->get_parameter("odom_topic").as_string();
    std::string output_path_topic = this->get_parameter("output_path_topic").as_string();
    std::string output_pose_topic = this->get_parameter("output_pose_topic").as_string();
    double transform_time_buffer = this->get_parameter("transform_time_buffer").as_double();
    double path_publish_frequency = this->get_parameter("path_publish_frequency").as_double();
    double path_sampling_frequency = this->get_parameter("path_sampling_frequency").as_double();
    output_file_ = this->get_parameter("output_file").as_string();
    input_file_ = this->get_parameter("input_file").as_string();
    save_path_ena_ = this->get_parameter("save_path_ena").as_bool();
    load_path_ena_ = this->get_parameter("load_path_ena").as_bool();
    sample_path_ena_ = this->get_parameter("sample_path_ena").as_bool();
    enable_button_used_ = this->get_parameter("enable_button_used").as_bool();


    if (load_path_ena_)
    {
      load_path_from_file(input_file_);
    }

    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10, std::bind(&PathManagerNode::odom_callback, this, std::placeholders::_1));
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(output_path_topic, 10);
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(output_pose_topic, 10);
    if (enable_button_used_)
    {
      button_subscriber_ = this->create_subscription<std_msgs::msg::Int8>(
        "/enable_0", 10, std::bind(&PathManagerNode::button_callback, this, std::placeholders::_1));
    }

    transform_time_buffer_ = tf2::durationFromSec(transform_time_buffer);

    path_publish_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / path_publish_frequency),
      std::bind(&PathManagerNode::publish_path, this));
    path_sampling_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / path_sampling_frequency),
      std::bind(&PathManagerNode::sample_path, this));
  }

  ~PathManagerNode()
  {
    if (!save_path_ena_)
      return;

    if (path_.poses.size() == 0)
    {
      RCLCPP_WARN(this->get_logger(), "Path is empty, not saving to file: %s", output_file_.c_str());
      return;
    }

    std::ofstream output(output_file_);

    if (!output.is_open())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open file to save path: %s", output_file_.c_str());
      return;
    }

    for (const auto &pose : path_.poses)
    {
      output << pose.header.stamp.sec << "." << pose.header.stamp.nanosec << ",";
      output << pose.pose.position.x << "," << pose.pose.position.y << "," << pose.pose.position.z << ",";
      output << pose.pose.orientation.x << "," << pose.pose.orientation.y << ",";
      output << pose.pose.orientation.z << "," << pose.pose.orientation.w << std::endl;
    }

    output.close();
  }

private:
  void button_callback(const std_msgs::msg::Int8::SharedPtr msg)
  {
    button_enabled_pressed_= msg->data;
  }

  void load_path_from_file(const std::string &filename)
  {
    std::ifstream input(filename);
    std::string line;

    if (!input.is_open())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open file to load path: %s", filename.c_str());
      return;
    }

    while (std::getline(input, line))
    {
      std::istringstream iss(line);
      geometry_msgs::msg::PoseStamped pose;
      double timestamp;
      char delimiter;

      iss >> timestamp >> delimiter;
      pose.header.stamp.sec = static_cast<int>(timestamp);
      pose.header.stamp.nanosec = static_cast<uint32_t>((timestamp - pose.header.stamp.sec) * 1e9);
      pose.header.frame_id = "map";

      iss >> pose.pose.position.x >> delimiter >> pose.pose.position.y >> delimiter >> pose.pose.position.z >> delimiter;
      iss >> pose.pose.orientation.x >> delimiter >> pose.pose.orientation.y >> delimiter;
      iss >> pose.pose.orientation.z >> delimiter >> pose.pose.orientation.w;

      path_.poses.push_back(pose);
    }
    path_.header.stamp.sec = path_.poses[-1].header.stamp.sec;
    path_.header.stamp.nanosec = path_.poses[-1].header.stamp.nanosec;
    path_.header.frame_id = "map";

    input.close();
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped input_pose;
    input_pose.header = msg->header;
    input_pose.pose = msg->pose.pose;

    try
    {
      buffer_.transform(input_pose, transformed_pose_, "map", transform_time_buffer_);
      //path_.header = transformed_pose_.header;
      //path_.poses.push_back(transformed_pose_);
      //publish_path();
      pose_publisher_->publish(transformed_pose_);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
    }
  }

  void sample_path()
  {
    if (!sample_path_ena_ || (enable_button_used_ && !button_enabled_pressed_))
      return;

    path_.header = transformed_pose_.header;
    path_.poses.push_back(transformed_pose_);
  }

  void publish_path()
  {
    path_publisher_->publish(path_);
    RCLCPP_INFO(this->get_logger(), "Published path with %d poses", path_.poses.size());
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr button_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::TimerBase::SharedPtr path_publish_timer_;
  rclcpp::TimerBase::SharedPtr path_sampling_timer_;
  nav_msgs::msg::Path path_;
  geometry_msgs::msg::PoseStamped transformed_pose_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  tf2::Duration transform_time_buffer_;
  std::string output_file_;
  std::string input_file_;
  bool save_path_ena_;
  bool load_path_ena_;
  bool sample_path_ena_; 
  bool enable_button_used_;
  bool button_enabled_pressed_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


#include "rclcpp/rclcpp.hpp"
#include "joy/joy.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class ManualControlNode : public rclcpp::Node {
private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_pub_;

  bool button_pressed_;

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy) {
    button_pressed_ = joy->buttons[4];

    if (button_pressed_)
        return;
    ackermann_msgs::msg::AckermannDriveStamped ackermann_msg;
    ackermann_msg.header.stamp = this->now();
    ackermann_msg.header.frame_id = "base_link";

    // Map joystick axes to servo and throttle values
    ackermann_msg.drive.speed = (-joy->axes[5] * 0.5 + 0.5) * 2.0;
    if (joy->axes[2] != 1.0) {
        ackermann_msg.drive.speed = (-joy->axes[2] * 0.5 + 0.5) * -2.0;
    }
    ackermann_msg.drive.steering_angle = -joy->axes[0] * -0.37;

    ackermann_pub_->publish(ackermann_msg);
  }

  void driveCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr drive) {
    if (button_pressed_) {
      ackermann_pub_->publish(*drive);
    }
  }

public:
  ManualControlNode() : Node("manual_control_node"), button_pressed_(false) {
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&ManualControlNode::joyCallback, this, std::placeholders::_1));
    drive_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      "/drive", 10, std::bind(&ManualControlNode::driveCallback, this, std::placeholders::_1));
    ackermann_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/ackermann_cmd", 10);
  }
};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManualControlNode>());
  rclcpp::shutdown();
  return 0;
}
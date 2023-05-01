#include "rclcpp/rclcpp.hpp"
#include "joy/joy.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

int map_450_to_360(float n) {
  return int((n / 360) * 450);
}

class JoystickControlNode : public rclcpp::Node {
private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr servo_pub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_pub_;

  float *ranges;

  float mean_of_ranges_from_to(int start, int end) const{
    float sum = 0;
    int count = 0;
    start = map_450_to_360(start);
    end = map_450_to_360(end);
    for (int i = start; i < end; i++){
      if (ranges[i] != std::numeric_limits<float>::infinity()){
        //if it is nan discard
        if (ranges[i] != ranges[i])
          ranges[i] = 0;
        sum += ranges[i];
        count++;
      }
    }
    return sum / count;
  }

  float get_range_at(float angle) const {
    return ranges[map_450_to_360(angle)];
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy) const {
    float limit = 0.5;
    float steer_limit = 0.3;

    float diff_means;
    float mean_means;

    ackermann_msgs::msg::AckermannDriveStamped ackermann_msg;
    ackermann_msg.header.stamp = this->now();
    ackermann_msg.header.frame_id = "base_link";

    // Map joystick axes to servo and throttle values
    ackermann_msg.drive.speed = (-joy->axes[5] * 0.5 + 0.5) * 2.0;
    if (joy->axes[2] != 1.0) {
      ackermann_msg.drive.speed = (-joy->axes[2] * 0.5 + 0.5) * -2.0;
    }
    ackermann_msg.drive.steering_angle = -joy->axes[0] * 0.37;

    if (joy->buttons[4]) {
      diff_means = mean_of_ranges_from_to(0, 50) - mean_of_ranges_from_to(130, 180);
      mean_means = (mean_of_ranges_from_to(0, 50) + mean_of_ranges_from_to(130, 180)) / 2;
      if (diff_means/mean_means > limit) {
        ackermann_msg.drive.steering_angle = steer_limit / 2;
      }
      else if (diff_means/mean_means < -limit) {
        ackermann_msg.drive.steering_angle = -steer_limit / 2;
      }
      else {
        ackermann_msg.drive.steering_angle = 0.0;
      }
      ackermann_msg.drive.speed = 0.2;
    }

    ackermann_pub_->publish(ackermann_msg);
    // std_msgs::msg::Float64servo_msg.data = ackermann_msg.drive.steering_angle;

    // servo_msg.data = ackermann_msg.drive.steering_angle;
    // // Publish messages
    // servo_pub_->publish(servo_msg); servo_msg;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) const {
    for (int i = 0; i < 450; i++) {
      ranges[i] = scan->ranges[i];
    }
  }

public:
  JoystickControlNode() : Node("joystick_control_node") {
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&JoystickControlNode::joyCallback, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&JoystickControlNode::scanCallback, this, std::placeholders::_1));
    //servo_pub = this->create_publisher<std_msgs::msg::Float64>("/commands/servo/position", 10);
    ackermann_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/ackermann_cmd", 10);
    ranges = new float[450];
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoystickControlNode>());
  rclcpp::shutdown();
  return 0;
}


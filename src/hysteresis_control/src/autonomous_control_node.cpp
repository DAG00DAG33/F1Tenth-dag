#include "rclcpp/rclcpp.hpp"
#include "joy/joy.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

int map_450_to_360(float n) {
  return int((n / 360) * 450);
}

class AutonomousControlNode : public rclcpp::Node {
private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

  float *ranges;

  float mean_of_ranges_from_to(int start, int end) const {
    float sum = 0;
    int count = 0;
    start = map_450_to_360(start);
    end = map_450_to_360(end);
    for (int i = start; i < end; i++) {
      if (ranges[i] != std::numeric_limits<float>::infinity()) {
        // if it is nan discard
        if (ranges[i] != ranges[i])
          ranges[i] = 0;
        sum += ranges[i];
        count++;
      }
    }
    return sum / count;
  }

  void autonomousDriving() {
    rclcpp::Rate loop_rate(10);

    while (rclcpp::ok()) {
      float limit = 0.5;
      float steer_limit = 0.3;

      float diff_means;
      float mean_means;

      ackermann_msgs::msg::AckermannDriveStamped drive_msg;
      drive_msg.header.stamp = this->now();
      drive_msg.header.frame_id = "base_link";

      diff_means = mean_of_ranges_from_to(0, 50) - mean_of_ranges_from_to(130, 180);
      mean_means = (mean_of_ranges_from_to(0, 50) + mean_of_ranges_from_to(130, 180)) / 2;
      if (diff_means / mean_means > limit) {
        drive_msg.drive.steering_angle = steer_limit / 2;
      } else if (diff_means / mean_means < -limit) {
        drive_msg.drive.steering_angle = -steer_limit / 2;
      } else {
        drive_msg.drive.steering_angle = 0.0;
      }
      drive_msg.drive.speed = 0.2;

      drive_pub_->publish(drive_msg);
      loop_rate.sleep();
    }
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) const {
    for (int i = 0; i < 450; i++) {
      ranges[i] = scan->ranges[i];
    }
  }

public:
  AutonomousControlNode() : Node("autonomous_control_node") {
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&AutonomousControlNode::scanCallback, this, std::placeholders::_1));
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
    ranges = new float[450];

    autonomousDriving();
  }
};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutonomousControlNode>());
  rclcpp::shutdown();
  return 0;
}
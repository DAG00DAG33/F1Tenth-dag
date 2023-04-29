
#include "rclcpp/rclcpp.hpp"
#include "joy/joy.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


int map_450_to_360(float n){
  return int((n / 360) * 450);
}


class JoystickControlNode : public rclcpp::Node{
private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr servo_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr throttle_pub_;

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

  float get_range_at(float angle){
    std::cout << "angle: " << angle << " map: " << map_450_to_360(angle) << " range: " << ranges[map_450_to_360(angle)] << std::endl;
    return ranges[map_450_to_360(angle)];
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy) const{
    std_msgs::msg::Float64 servo_msg;
    std_msgs::msg::Float64 throttle_msg;

    float limit = 0.5;
    float steer_limit = 0.7;

    float diff_means;
    float mean_means;

    // Map joystick axes to servo and throttle values
    throttle_msg.data = (-joy->axes[5] * 0.5 + 0.5) * 4000;
    if (joy->axes[2] != 1.0)
      throttle_msg.data = - (-joy->axes[2] * 0.5 + 0.5) * 4000;
    servo_msg.data = -joy->axes[0] * 0.5 + 0.5;
    


    if (joy->buttons[4])
    {
    diff_means = mean_of_ranges_from_to(0, 50) - mean_of_ranges_from_to(130, 180);
    mean_means = (mean_of_ranges_from_to(0, 50) + mean_of_ranges_from_to(130, 180)) / 2;
      if (diff_means/mean_means > limit){
        servo_msg.data = 0.5 + steer_limit / 2;
      }
      else if (diff_means/mean_means < -limit){
        servo_msg.data = 0.5 - steer_limit / 2;
      }
      else{
        servo_msg.data = 0.5;
      }

      std::cout << "diff_means: " << diff_means << " mean_means: " << mean_means << " " << mean_of_ranges_from_to(0, 50) << " " << mean_of_ranges_from_to(130, 180) << "ranges[360]" << ranges[360 - 1] << std::endl;

      throttle_msg.data = 800;
    }
    //std::cout << "0: " << get_range_at(0) << " 90: " << get_range_at(90) << " 180: " << get_range_at(180) << " 270: " << get_range_at(270) << " 360: " << get_range_at(360 - 1) << std::endl;

    // Publish messages
    servo_pub_->publish(servo_msg);
    throttle_pub_->publish(throttle_msg);
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) const{
    for (int i = 0; i < 450; i++)
      ranges[i] = scan->ranges[i];
  }

public:

  JoystickControlNode() : Node("joystick_control_node"){
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&JoystickControlNode::joyCallback, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&JoystickControlNode::scanCallback, this, std::placeholders::_1));
    servo_pub_ = this->create_publisher<std_msgs::msg::Float64>("/commands/servo/position", 10);
    throttle_pub_ = this->create_publisher<std_msgs::msg::Float64>("/commands/motor/speed", 10);
    ranges = new float[450];
  }


  


};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoystickControlNode>());
  rclcpp::shutdown();
  return 0;
}

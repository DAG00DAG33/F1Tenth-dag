#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include "std_msgs/msg/int8.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

class ImuToOdometryNode : public rclcpp::Node
{
public:
    ImuToOdometryNode()
        : Node("imu_to_odometry_node")
    {
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", 10, std::bind(&ImuToOdometryNode::imuCallback, this, std::placeholders::_1));
        reset_speed_sub_ = this->create_subscription<std_msgs::msg::Int8>("enable_1", 10, std::bind(&ImuToOdometryNode::resetCallback, this, std::placeholders::_1));

        velocity_.x = 0.0;
        velocity_.y = 0.0;
        velocity_.z = 0.0;

        position_.x = 0.0;
        position_.y = 0.0;
        position_.z = 0.0;

        orientation_.x = 0.0;
        orientation_.y = 0.0;
        orientation_.z = 0.0;
        orientation_.w = 1.0;

        angle_ = 0.0;

        last_time_ = this->now();


        avg_start_time_ = this->now();

        avg_data_count_ = 0;
        avg_sum_linear_acceleration_.x = 0;
        avg_sum_linear_acceleration_.y = 0;
        avg_sum_linear_acceleration_.z = 0;
        avg_sum_angular_velocity_ = 0;

        avg_calculated_ = false;

        tf_pub_.reset(new tf2_ros::TransformBroadcaster(this));

    }

private:

    void resetCallback(const std_msgs::msg::Int8::SharedPtr msg)
    {
        if (msg->data == 1)
        {
            velocity_.x = 0.0;
            velocity_.y = 0.0;
            velocity_.z = 0.0;

            // angle_ = 0.0;

            // last_time_ = this->now();

            // avg_start_time_ = this->now();

            // avg_data_count_ = 0;
            // avg_sum_linear_acceleration_.x = 0;
            // avg_sum_linear_acceleration_.y = 0;
            // avg_sum_linear_acceleration_.z = 0;
            // avg_sum_angular_velocity_ = 0;

            // avg_calculated_ = false;
        }
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        auto current_time = this->now();
        auto time_delta = (current_time - last_time_).seconds();

        // During first second calculate averages
        if (!avg_calculated_ && (current_time - avg_start_time_).seconds() < 25.0)
        {
            avg_data_count_++;

            avg_sum_linear_acceleration_.x += msg->linear_acceleration.x;
            avg_sum_linear_acceleration_.y += msg->linear_acceleration.y;
            avg_sum_linear_acceleration_.z += msg->linear_acceleration.z;
            avg_sum_angular_velocity_ += msg->angular_velocity.z;
            return;
        }
        else if (!avg_calculated_ && avg_data_count_ > 0) // After first second calculate averages and print them
        {
            avg_linear_acceleration_.x = avg_sum_linear_acceleration_.x / avg_data_count_;
            avg_linear_acceleration_.y = avg_sum_linear_acceleration_.y / avg_data_count_;
            avg_linear_acceleration_.z = avg_sum_linear_acceleration_.z / avg_data_count_;
            avg_angular_velocity_ = avg_sum_angular_velocity_ / avg_data_count_;

            RCLCPP_INFO(this->get_logger(), "Average Linear Acceleration (x, y, z): %f, %f, %f", avg_linear_acceleration_.x, avg_linear_acceleration_.y, avg_linear_acceleration_.z);
            RCLCPP_INFO(this->get_logger(), "Average Angular Velocity (z): %f", avg_angular_velocity_);

            avg_calculated_ = true;
            last_time_ = this->now();
            velocity_.x = 0.0;
            velocity_.y = 0.0;
            velocity_.z = 0.0;

            position_.x = 0.0;
            position_.y = 0.0;
            position_.z = 0.0;
            return;
        }

        // Integrate linear acceleration to get velocity
        double linear_acceleration_x = msg->linear_acceleration.x - avg_linear_acceleration_.x;
        double linear_acceleration_y = msg->linear_acceleration.y - avg_linear_acceleration_.y;
        velocity_.x += (linear_acceleration_x * sin(angle_) + linear_acceleration_y * cos(angle_)) * time_delta;
        velocity_.y += (-linear_acceleration_x * cos(angle_) + linear_acceleration_y * sin(angle_)) * time_delta;
        // velocity_.z += (msg->linear_acceleration.z -avg_linear_acceleration_.z) * time_delta;

        //decay rate
        velocity_.x *= 0.999;
        velocity_.y *= 0.999;

        //RCLCPP_INFO(this->get_logger(), "Pos: %f, %f", position_.x, position_.y);
        RCLCPP_INFO(this->get_logger(), "Vel: %f, %f", velocity_.x, velocity_.y);

        // Integrate velocity to get position
        position_.x += velocity_.x * time_delta;
        position_.y += velocity_.y * time_delta;
        // position_.z += velocity_.z * time_delta;


        // Integrate angular velocity to get orientation (quaternion)
        // Assuming IMU is horizontal (z-axis pointing up)
        //double delta_angle = msg->angular_velocity.z * time_delta;
        //orientation_.z = sin(delta_angle / 2);
        //orientation_.w = cos(delta_angle / 2);
        angle_ -= (msg->angular_velocity.z - avg_angular_velocity_) * time_delta;
        orientation_.z = sin(angle_ / 2);
        orientation_.w = cos(angle_ / 2);

        // Create and publish odometry message
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        //odom_msg.pose.pose.position = position_;
        odom_msg.pose.pose.position.x = position_.x;
        odom_msg.pose.pose.position.y = position_.y;
        odom_msg.pose.pose.position.z = 0; //position_.z;
        //odom_msg.pose.pose.orientation = orientation_;
        odom_msg.pose.pose.orientation.x = orientation_.x;
        odom_msg.pose.pose.orientation.y = orientation_.y;
        odom_msg.pose.pose.orientation.z = orientation_.z;
        odom_msg.pose.pose.orientation.w = orientation_.w;
        //odom_msg.twist.twist.linear = velocity_;
        odom_msg.twist.twist.linear.x = velocity_.x;
        odom_msg.twist.twist.linear.y = velocity_.y;
        odom_msg.twist.twist.linear.z = 0;
        odom_msg.twist.twist.angular.x = 0;
        odom_msg.twist.twist.angular.y = 0;
        odom_msg.twist.twist.angular.z = msg->angular_velocity.z;

        odom_pub_->publish(odom_msg);

        geometry_msgs::msg::TransformStamped tf;
        tf.header.frame_id = "odom";
        tf.child_frame_id = "base_link";
        tf.header.stamp = current_time;
        tf.transform.translation.x = position_.x;
        tf.transform.translation.y = position_.y;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation = odom_msg.pose.pose.orientation;

        if (rclcpp::ok()) {
            tf_pub_->sendTransform(tf);
        }

        last_time_ = current_time;
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr reset_speed_sub_;
    rclcpp::Time last_time_;
    geometry_msgs::msg::Vector3 velocity_;
    geometry_msgs::msg::Vector3 position_;
    geometry_msgs::msg::Quaternion orientation_;

    double  angle_;

    bool avg_calculated_;
    rclcpp::Time avg_start_time_;
    size_t avg_data_count_;
    geometry_msgs::msg::Vector3 avg_sum_linear_acceleration_;
    double avg_sum_angular_velocity_;
    geometry_msgs::msg::Vector3 avg_linear_acceleration_;
    double avg_angular_velocity_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuToOdometryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


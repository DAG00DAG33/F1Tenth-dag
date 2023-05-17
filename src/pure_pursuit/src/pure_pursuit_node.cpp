/*
#include "pure_pursuit_node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
*/
#include <cmath>

#include "rclcpp/rclcpp.hpp"
//#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
//#include <geometry_msgs/msg/pose_stamped.hpp>
//#include <tf2_ros/transform_listener.h>
//#include <tf2_ros/buffer.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"


class PurePursuitNode : public rclcpp::Node
{
public:
    PurePursuitNode() : Node("pure_pursuit_node"), current_pose_received_(false)
    {
        // Initialize ROS communication
        current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/map_pose", 10, std::bind(&PurePursuitNode::currentPoseCallback, this, std::placeholders::_1));
        target_path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/target_path", 10, std::bind(&PurePursuitNode::targetPathCallback, this, std::placeholders::_1));
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        lookahead_point_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/lookahead_point", 10);
        closest_point_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/closest_point", 10);

        // Initialize parameters
        this->declare_parameter<double>("wheelbase", 0.3);
        this->declare_parameter<double>("lookahead_distance", 1.0);
        this->declare_parameter<std::string>("car_frame", "base_link");
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        car_frame_ = this->get_parameter("car_frame").as_string();
        lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
    }

private:

    void currentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose_ = *msg;
        current_pose_received_ = true;
        calculateControl();
    }

    void targetPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        target_path_ = *msg;
        if (current_pose_received_)
            calculateControl();
    }

    void calculateControl()
    {
        // calculate control
        // TODO: Implement the pure pursuit algorithm here
        // 1. Find closest point on path to current pose
        // 2. Find lookahead point on path
        // 3. Calculate curvature
        // 4. Use curvature to calculate steering angle
        // 5. Publish AckermannDriveStamped message

        // find the closest point on the path
        geometry_msgs::msg::Point closest_point;
        double min_distance = std::numeric_limits<double>::max();
        int closest_point_index = 0;
        for (unsigned int i=0; i < target_path_.poses.size(); i++)
        {
            double distance = calculateDistance(current_pose_.pose.position, target_path_.poses[i].pose.position);
            if (distance < min_distance)
            {
                min_distance = distance;
                closest_point_index = i;
                closest_point = target_path_.poses[i].pose.position;
            }
        }
        
        
        // find the lookahead point on the path
        geometry_msgs::msg::Point lookahead_point = getLookaheadPoint(closest_point, closest_point_index, lookahead_distance_); // we use wheelbase as the lookahead distance for simplicity
        closest_point_pub_->publish(closest_point);
        lookahead_point_pub_->publish(lookahead_point);
        
        // transform the lookahead point to the robot's frame
        geometry_msgs::msg::Point transformed_lookahead_point = transformPoint(lookahead_point, current_pose_);
        
        // calculate the curvature
        double radius = calculateRadius(transformed_lookahead_point);
        
        // calculate the steering angle based on the curvature
        double steering_angle = std::atan(wheelbase_ / radius);

        // create and publish the AckermannDriveStamped message
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = this->now();
        drive_msg.header.frame_id = car_frame_;
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = 1.0;  // let's use a constant speed for simplicity
        
        drive_pub_->publish(drive_msg);
    }

    double calculateDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2)
    {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        double dz = p1.z - p2.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    geometry_msgs::msg::Point getLookaheadPoint(const geometry_msgs::msg::Point& closest_point, unsigned int closest_point_index, double lookahead_distance)
    {
        // TODO: Implement method to find lookahead point given closest point on path and lookahead distance
        // For simplicity, we'll assume the path is represented as a sequence of points.
        
        // Start from the closest point on the path
        for (unsigned int i = closest_point_index; i < target_path_.poses.size(); i++)
        {
            if (calculateDistance(closest_point, target_path_.poses[i].pose.position) >= lookahead_distance)
            {
                return target_path_.poses[i].pose.position;
            }
        }

        for (unsigned int i = 0; i < closest_point_index; i++)
        {
            if (calculateDistance(closest_point, target_path_.poses[i].pose.position) >= lookahead_distance)
            {
                return target_path_.poses[i].pose.position;
            }
        }
        
        RCLCPP_WARN(this->get_logger(), "Lookahead point beyond path");
        return target_path_.poses.back().pose.position;
    }

    geometry_msgs::msg::Point transformPoint(const geometry_msgs::msg::Point& point, const geometry_msgs::msg::PoseStamped& pose)
    {
        // TODO: Implement method to transform point from world frame to robot frame
        // create a transformation matrix
        tf2::Transform tf2_pose;
        tf2::fromMsg(pose.pose, tf2_pose);
        
        // create a tf2 point to transform
        tf2::Vector3 tf2_point(point.x, point.y, point.z);
        
        // transform the point to the robot's frame
        tf2::Vector3 tf2_transformed_point = tf2_pose.inverse() * tf2_point;
        
        // convert back to geometry_msgs
        geometry_msgs::msg::Point transformed_point;
        transformed_point.x = tf2_transformed_point.x();
        transformed_point.y = tf2_transformed_point.y();
        transformed_point.z = tf2_transformed_point.z();
        
        return transformed_point;
    }

    double calculateRadius(const geometry_msgs::msg::Point& point)
    {
        return point.y / (1 - cos(M_PI - 2*std::atan(std::abs(point.x) / point.y)));
    };

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr target_path_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr lookahead_point_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr closest_point_pub_;

    geometry_msgs::msg::PoseStamped current_pose_;
    bool current_pose_received_;
    nav_msgs::msg::Path target_path_;
    double wheelbase_;
    double lookahead_distance_;
    std::string car_frame_;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PurePursuitNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
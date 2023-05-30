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
#include "sensor_msgs/msg/laser_scan.hpp"

double linear_map(const double min_in, const double max_in, const double mint_out, const double max_out, double x){
    return (x - min_in) * (max_out - mint_out) / (max_in - min_in) + mint_out;
}

int map_450_to_360(float n) {
  return int((n / 360) * 450);
}



class PurePursuitNode : public rclcpp::Node
{
public:
    PurePursuitNode() : Node("pure_pursuit_node"), current_pose_received_(false), target_received_(false)
    {
        //this->declare_parameter<double>("constant_throttle", 1.0);
        this->declare_parameter<double>("max_throttle", 3.0);
        this->declare_parameter<double>("max_throttle_radius", 9.0);
        this->declare_parameter<double>("min_throttle", 1.5);
        this->declare_parameter<double>("min_throttle_radius", 2.25);
        this->declare_parameter<double>("throttle_parameter", 1.0);
        this->declare_parameter<double>("throttle_lookahead_multiply", 2.0);

        this->declare_parameter<double>("wheelbase", 0.25);
        this->declare_parameter<double>("initial_lookahead_distance", 0.6);
        this->declare_parameter<double>("lookahead_parameter", 0.5);
        this->declare_parameter<std::string>("car_frame", "base_link");
        this->declare_parameter<std::string>("drive_topic", "/drive");
        this->declare_parameter<std::string>("target_topic", "/target_path");
        this->declare_parameter<std::string>("current_pose_topic", "/map_pose");


        //constant_throttle_ = this->get_parameter("constant_throttle").as_double();
        max_throttle_ = this->get_parameter("max_throttle").as_double();
        max_throttle_radius_ = this->get_parameter("max_throttle_radius").as_double();
        min_throttle_ = this->get_parameter("min_throttle").as_double();
        min_throttle_radius_ = this->get_parameter("min_throttle_radius").as_double();
        throttle_parameter_ = this->get_parameter("throttle_parameter").as_double();
        throttle_lookahead_multiply_ = this->get_parameter("throttle_lookahead_multiply").as_double();

        wheelbase_ = this->get_parameter("wheelbase").as_double();
        lookahead_distance_ = this->get_parameter("initial_lookahead_distance").as_double();
        lookahead_parameter_ = this->get_parameter("lookahead_parameter").as_double();
        car_frame_ = this->get_parameter("car_frame").as_string();
        std::string drive_topic = this->get_parameter("drive_topic").as_string();
        std::string target_topic = this->get_parameter("target_topic").as_string();
        std::string current_pose_topic = this->get_parameter("current_pose_topic").as_string();

        current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            current_pose_topic, 10, std::bind(&PurePursuitNode::currentPoseCallback, this, std::placeholders::_1));
        target_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            target_topic, 10, std::bind(&PurePursuitNode::targetPathCallback, this, std::placeholders::_1));
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&PurePursuitNode::scanCallback, this, std::placeholders::_1));
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
        lookahead_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/lookahead_point", 10);
        closest_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/closest_point", 10);

        float controler_period = 0.05;
        control_timer_ = this->create_wall_timer(std::chrono::duration<double>(controler_period), std::bind(&PurePursuitNode::calculateControl, this));

        ranges_ = new double[450];
    }

private:

    float mean_of_ranges_from_to(int start, int end) const {
        float sum = 0;
        int count = 0;
        start = map_450_to_360(start);
        end = map_450_to_360(end);
        for (int i = start; i < end; i++) {
        if (ranges_[i] != std::numeric_limits<float>::infinity()) {
            // if it is nan discard
            if (ranges_[i] != ranges_[i])
                ranges_[i] = 0;
            sum += ranges_[i];
            count++;
        }
        }
        return sum / count;
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) const {
        for (int i = 0; i < 450; i++) {
            ranges_[i] = scan->ranges[i];
        }
        //RCLCPP_INFO(this->get_logger(), "scanCallback");
    }

    void currentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose_ = *msg;
        current_pose_received_ = true;
        //RCLCPP_INFO(this->get_logger(), "pose received");
        // if (target_received_)
        //     calculateControl();
    }

    void targetPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        target_path_ = *msg;
        target_received_ = true;
        //RCLCPP_INFO(this->get_logger(), "target received");
        // if (current_pose_received_)
        //     calculateControl();
    }

    void calculateControl()
    {
        // find the closest point on the path
        if (!target_received_ || !current_pose_received_)
            return;
        geometry_msgs::msg::Point closest_point;
        int closest_point_index;
        double min_distance = std::numeric_limits<double>::max();
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
        geometry_msgs::msg::PointStamped lookahead_point_stamped;
        lookahead_point_stamped.point = lookahead_point;
        lookahead_point_stamped.header.stamp = this->now();
        lookahead_point_stamped.header.frame_id = "map";
        geometry_msgs::msg::PointStamped closest_point_stamped;
        closest_point_stamped.point = closest_point;
        closest_point_stamped.header.stamp = this->now();
        closest_point_stamped.header.frame_id = "map";

        closest_point_pub_->publish(closest_point_stamped);
        lookahead_point_pub_->publish(lookahead_point_stamped);
        
        // transform the lookahead point to the robot's frame
        geometry_msgs::msg::Point transformed_lookahead_point = transformPoint(lookahead_point, current_pose_);
        
        // calculate the curvature
        double radius = calculateRadius(transformed_lookahead_point);
        //RCLCPP_INFO(this->get_logger(), "radius: %f", radius);
        

        // lookahead for the speed
        geometry_msgs::msg::Point transformed_lookahead_point_speed = transformPoint(getLookaheadPoint(closest_point, closest_point_index, lookahead_distance_ * throttle_lookahead_multiply_), current_pose_);
        double speed = calculateSpeed(transformed_lookahead_point_speed);
        lookahead_distance_ = lookahead_parameter_ * speed;

        if (std::abs(radius) > 3.0 && (mean_of_ranges_from_to(80, 100) < lookahead_distance_ * 1.6 || mean_of_ranges_from_to(80, 90) < lookahead_distance_ * 1.6 || mean_of_ranges_from_to(90, 100) < lookahead_distance_ * 1.6)){
            speed *= linear_map(0, lookahead_distance_, 0, 0.2, mean_of_ranges_from_to(80, 100));//mean_of_ranges_from_to(80, 100);
            lookahead_distance_ = mean_of_ranges_from_to(80, 100);
            RCLCPP_INFO(this->get_logger(), "Obstacle detected at %f, with lookahead %f, so speed is %f", mean_of_ranges_from_to(85, 95), lookahead_distance_, speed);
            if (mean_of_ranges_from_to(80, 90) > mean_of_ranges_from_to(90, 100))
                radius = -0.5;
            else   
                radius = 0.5;
        }
        else{
            RCLCPP_INFO(this->get_logger(), "Obstacle NOT detected at %f, with lookahead %f, so speed is %f", mean_of_ranges_from_to(85, 95), lookahead_distance_, speed);
            
        }

        // calculate the steering angle based on the curvature
        double steering_angle = std::atan(wheelbase_ / radius);


        // create and publish the AckermannDriveStamped message
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = this->now();
        drive_msg.header.frame_id = car_frame_;
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = speed;
        
        drive_pub_->publish(drive_msg);
        //RCLCPP_INFO(this->get_logger(), "drive published");
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
        //return point.y / (1 - cos(M_PI - 2*std::atan(std::abs(point.x) / point.y)));
        double true_x = point.x + 0.25; //adding wheelbase
        double dis_sqr = true_x*true_x + point.y*point.y;
        return dis_sqr / (2*point.y);
    };

    double calculateSpeed(geometry_msgs::msg::Point transformed_lookahead_point)
    {
        double radius = calculateRadius(transformed_lookahead_point);
        if (std::abs(radius) < min_throttle_radius_ || transformed_lookahead_point.x < 0.0)
            return min_throttle_;
        if (std::abs(radius) > max_throttle_radius_)
            return max_throttle_;
        return std::sqrt(throttle_parameter_ * std::abs(radius));
        

    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr target_path_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr lookahead_point_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr closest_point_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    geometry_msgs::msg::PoseStamped current_pose_;
    bool current_pose_received_;
    bool target_received_;
    nav_msgs::msg::Path target_path_;
    double wheelbase_;
    std::string car_frame_;
    //double constant_throttle_;
    double max_throttle_;
    double max_throttle_radius_;
    double min_throttle_;
    double min_throttle_radius_;
    double throttle_parameter_;
    double throttle_lookahead_multiply_;
    double lookahead_distance_;
    double lookahead_parameter_;
    double *ranges_;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PurePursuitNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
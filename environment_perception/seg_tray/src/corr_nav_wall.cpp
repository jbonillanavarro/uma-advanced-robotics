#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <cmath>
#include <vector>
#include <numeric>

class CorridorNavigationNode : public rclcpp::Node
{
public:
    CorridorNavigationNode() : Node("corr_nav_node")
    {
        // Load parameters
        loadParameters();

        RCLCPP_INFO(this->get_logger(), "CorridorNavigationNode started");

        // Publisher for cmd_vel
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/PioneerP3DX/cmd_vel", 10);
        // Subscriber for robot pose
        pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/PioneerP3DX/odom", 10,
            std::bind(&CorridorNavigationNode::odomCallback, this, std::placeholders::_1));

        // Subscriber for laser scan data
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/PioneerP3DX/laser_scan", 10,
            std::bind(&CorridorNavigationNode::laserCallback, this, std::placeholders::_1));

        // Timer to publish velocity commands at regular intervals
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(time_step_),
            std::bind(&CorridorNavigationNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Corridor Navigation Node Initialized");
        measured_data_=false;
    }

private:
    void loadParameters()
    {
         // Declare parameters of this node (name, initial_value)
        this->declare_parameter("time_step", 25);  // in milliseconds
        this->declare_parameter("max_linear_speed", 1.2);
        this->declare_parameter("max_angular_speed", 2.0);
        this->declare_parameter("wheel_base", 0.331);
        this->declare_parameter("wheel_radius", 0.097518);
        this->declare_parameter("corridor_width", 10.0);  // meters
        this->declare_parameter("look_ahead_distance", 1.0);  // meters 
        // Read parameters
        time_step_ = this->get_parameter("time_step").as_int();
        max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
        max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
        wheel_base_ = this->get_parameter("wheel_base").as_double();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        corridor_width_ = this->get_parameter("corridor_width").as_double();
        look_ahead_distance_ = this->get_parameter("look_ahead_distance").as_double();

        RCLCPP_INFO(this->get_logger(), 
            "max_linear_speed: %.2f, max_angular_speed: %.2f, wheel_base: %.2f, wheel_radius: %.2f, corridor_width: %.2f, look_ahead_distance: %.2f", 
            max_linear_speed_, max_angular_speed_, wheel_base_, wheel_radius_, corridor_width_, look_ahead_distance_);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        // Extract yaw from quaternion
        double siny_cosp = 2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
        double cosy_cosp = 1 - 2 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
        current_theta_ = std::atan2(siny_cosp, cosy_cosp);
    }

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        WallInfo left_wall = extractWall(msg, 20.0, M_PI/2);
        WallInfo right_wall = extractWall(msg, 20.0, -M_PI/2);

        if (left_wall.valid) {
            dist_left_ = left_wall.distance;
            angle_left_ = left_wall.angle;
        }
        if (right_wall.valid) {
            dist_right_ = right_wall.distance;
            angle_right_ = right_wall.angle;
        }
        measured_data_ = true; 
    }

    struct WallInfo {
        bool valid;
        double distance;
        double angle;
    };

    WallInfo extractWall(const sensor_msgs::msg::LaserScan::SharedPtr& msg, double window_size_deg, double center_angle_rad)
    {
        WallInfo wall;
        wall.valid = false;
        wall.distance = 0.0;
        wall.angle = 0.0;

        double window = window_size_deg * M_PI / 180;
        double angle_min = msg->angle_min;
        double angle_inc  = msg->angle_increment;
        int n = msg->ranges.size();

        double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_x2 = 0.0;
        int count = 0;

        for (int i = 0; i < n; i++) {
            double angle = angle_min + i * angle_inc;

            if (angle > center_angle_rad - window && angle < center_angle_rad + window)
            {
                double r = msg->ranges[i];

                if (std::isfinite(r) && r > msg->range_min && r < msg->range_max) {
                    double x = r * std::cos(angle);
                    double y = r * std::sin(angle);

                    sum_x += x;
                    sum_y += y;
                    sum_xy += (x * y);
                    sum_x2 += (x * x);
                    count++;
                }
            }
        }
        if (count > 5) {
            double denominator = (count * sum_x2) - (sum_x * sum_x);
            if (std::abs(denominator) > 1e-6) {
                double m = ((count * sum_xy) - (sum_x * sum_y)) / denominator;
                double b = (sum_y -(m * sum_x)) / count;

                wall.distance = std::abs(b) / (std::sqrt((m * m) + 1.0));
                wall.angle = std::atan(m);
                wall.valid = true;
            }
        }
        return wall;
    }

    void controlLoop()
    {
        if (!measured_data_) {
            RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for laser data...");
            return;
        }
        else {
            measured_data_=false;
            double theta = -(angle_left_ + angle_right_) / 2;
            double dl = (dist_left_ - dist_right_) / 2;
            double d = look_ahead_distance_;

            RCLCPP_INFO(this->get_logger(), "Running");

            double dx = (dl * std::sin(theta)) + (d * std::cos(theta));
            double dy = (dl * std::cos(theta)) - (d * std::sin(theta));

            double dist = std::hypot(dx, dy);
            double curve = (2 *dy) / (dist * dist);

            double vl = (max_linear_speed_ * (1 - (wheel_base_ / 2) * curve)) / wheel_radius_;
            double vr =(max_linear_speed_ * (1 + (wheel_base_ / 2) * curve)) / wheel_radius_;
            
            double linear_velocity = ((vl + vr) / 2) * wheel_radius_;
            double angular_velocity = ((vr - vl) / wheel_base_) * wheel_radius_;


            geometry_msgs::msg::Twist cmd_vel_msg;
            cmd_vel_msg.linear.x = linear_velocity;
            cmd_vel_msg.angular.z = angular_velocity;
            cmd_vel_pub_->publish(cmd_vel_msg);
        }
    }

    // Publishers and Subscribers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    int time_step_;
    double max_linear_speed_;
    double max_angular_speed_;      
    double wheel_base_;
    double wheel_radius_;
    double corridor_width_;
    double look_ahead_distance_;

    // Actual robot position
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_theta_ = 0.0;  

    // Laser data
    double dist_left_;
    double dist_right_;
    double angle_left_ = 0.0;
    double angle_right_ = 0.0;
    double theta_sign_ = 1.0; // +1 if facing left wall, -1 if facing right wall
    double measured_data_=false;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
};  

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CorridorNavigationNode>());
    rclcpp::shutdown();
    return 0;
}
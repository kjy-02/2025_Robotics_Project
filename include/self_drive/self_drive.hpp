#ifndef SELF_DRIVE_HPP_
#define SELF_DRIVE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class SelfDrive : public rclcpp::Node
{
public:
  SelfDrive();

private:
  // --- Constants ---
  const double TARGET_SPEED = 0.22;
  const double SAFE_DISTANCE = 0.4;
  const double KP_ANGULAR = 2.3;
  const double MAX_ANGULAR_VEL = 1.8;

  // --- Member Variables ---
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
  int step_counter_;

  // --- Member Functions ---
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  
  double get_scan_average(
    const sensor_msgs::msg::LaserScan::SharedPtr scan, 
    int center_angle_deg, 
    int range_deg);

  geometry_msgs::msg::Twist calculate_twist_command(
    double front_dist, 
    double left_dist, 
    double right_dist);
};

#endif  // SELF_DRIVE_HPP_

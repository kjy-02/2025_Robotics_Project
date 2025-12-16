#include "self_drive/self_drive.hpp"
#include <cmath>
#include <algorithm>

using std::placeholders::_1;

SelfDrive::SelfDrive() 
: Node("self_drive"), step_counter_(0)
{
  auto scan_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  scan_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", scan_qos, std::bind(&SelfDrive::scan_callback, this, _1));
  auto vel_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  
  vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", vel_qos);
}
// LiDAR 데이터 평균 계산
double SelfDrive::get_scan_average(
  const sensor_msgs::msg::LaserScan::SharedPtr scan, int center_angle, int window)
{
  double sum = 0.0;
  int count = 0;
  size_t size = scan->ranges.size();

  for (int i = -window; i <= window; ++i) {
    int idx = (center_angle + i + size) % size;
    float r = scan->ranges[idx];

    if (!std::isinf(r) && !std::isnan(r) && r > 0.01 && r < 3.0) {
      sum += r;
      count++;
    }
  }
  return (count == 0) ? 2.0 : (sum / count);
}

// 주행 로직
geometry_msgs::msg::Twist SelfDrive::calculate_twist_command(
  double front, double left, double right)
{
  geometry_msgs::msg::Twist twist;

  if (front < SAFE_DISTANCE) {
    twist.linear.x = 0.0;
    twist.angular.z = (left > right) ? 0.8 : -0.8;
  }
  else {
    twist.linear.x = TARGET_SPEED;
    
    double error = left - right;
    error = std::max(-1.0, std::min(1.0, error));

    twist.angular.z = error * KP_ANGULAR;
    twist.angular.z = std::max(-MAX_ANGULAR_VEL, std::min(MAX_ANGULAR_VEL, twist.angular.z));
  }
  return twist;
}

void SelfDrive::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  double front = get_scan_average(scan, 0, 10);
  double left = get_scan_average(scan, 45, 15);
  double right = get_scan_average(scan, 315, 15);

  auto twist = calculate_twist_command(front, left, right);

  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "base_link";
  msg.twist = twist;

  vel_pub_->publish(msg);

  RCLCPP_INFO(this->get_logger(), 
    "Step: %d | F: %.2f L: %.2f R: %.2f | Lin: %.2f Ang: %.2f",
    step_counter_++, front, left, right, twist.linear.x, twist.angular.z);
}

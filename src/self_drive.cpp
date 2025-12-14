#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp> 
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

class SelfDrive : public rclcpp::Node
{
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pose_pub_;
  int step_;

  // 상수 정의 (최적화된 값 적용)
  const float TARGET_SPEED = 0.22;     // 0.22 m/s
  const float SAFE_DISTANCE = 0.4;     // 0.4 m
  const float KP_ANGULAR = 2.3;        // 민감도 2.3

public:
  SelfDrive() : rclcpp::Node("self_drive"), step_(0)
  {
    auto lidar_qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
    lidar_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    
    auto callback = std::bind(&SelfDrive::subscribe_scan, this, std::placeholders::_1);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", lidar_qos_profile, callback);
        
    auto vel_qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
    pose_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", vel_qos_profile);
  }

  void subscribe_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    // 1. 센서 데이터 처리 (대각선 45도 감지)
    float front_dist = get_range_avg(scan, 0, 10);    
    float left_dist = get_range_avg(scan, 45, 15);    
    float right_dist = get_range_avg(scan, 315, 15);  

    // 2. 주행 명령 결정
    geometry_msgs::msg::TwistStamped vel_msg;
    vel_msg.header.stamp = this->now();
    vel_msg.header.frame_id = "base_link";

    decide_movement(vel_msg.twist, front_dist, left_dist, right_dist);

    // 3. 로그 출력 및 명령 발행
    RCLCPP_INFO(this->get_logger(), 
      "Step: %d | F: %.2f L: %.2f R: %.2f | Lin: %.2f Ang: %.2f", 
      step_, front_dist, left_dist, right_dist, vel_msg.twist.linear.x, vel_msg.twist.angular.z);
      
    pose_pub_->publish(vel_msg);
    step_++;
  }

private:
  // 특정 각도 범위의 평균 거리를 구하는 헬퍼 함수
  float get_range_avg(const sensor_msgs::msg::LaserScan::SharedPtr scan, int center_angle, int window)
  {
    int size = scan->ranges.size();
    float sum = 0.0;
    int count = 0;

    for (int i = -window; i <= window; i++)
    {
      int idx = (center_angle + i + size) % size;
      float r = scan->ranges[idx];

      if (!std::isinf(r) && !std::isnan(r) && r > 0.01 && r < 3.0) {
        sum += r;
        count++;
      }
    }

    if (count == 0) return 2.0; 
    return sum / count;
  }

  // 주행 알고리즘 핵심 로직
  void decide_movement(geometry_msgs::msg::Twist &twist_cmd, float front, float left, float right)
  {
    if (front < SAFE_DISTANCE)
    {
      twist_cmd.linear.x = 0.0; 
      if (left > right) {
        twist_cmd.angular.z = 0.8; 
      } else {
        twist_cmd.angular.z = -0.8; 
      }
    }
    else
    {
      twist_cmd.linear.x = TARGET_SPEED;
      float error = left - right;
      error = std::max(-1.0f, std::min(1.0f, error));
      twist_cmd.angular.z = error * KP_ANGULAR;
      twist_cmd.angular.z = std::max(-1.8f, std::min(1.8f, (float)twist_cmd.angular.z));
    }
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SelfDrive>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

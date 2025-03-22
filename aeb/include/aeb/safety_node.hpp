#ifndef SAFETY_NODE_HPP
#define SAFETY_NODE_HPP
#include <vector>
#include <Eigen/Dense> 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace AEB {

class SafetyNode : public rclcpp::Node {
public:
	SafetyNode();
	void brake();
private:
	float speed_;
	float heading_;
  float throttle_cmd_raw_;
	Eigen::ArrayXf ttc_map_;	
  bool enable_aeb_;  

	static constexpr double EPSILON = 0.0001f;	
	static constexpr size_t BEAM_COUNT = 1080;
	static constexpr float AEB_MIN_RADIUS = 0.05;

	static constexpr const char* SCAN_TOPIC = "autodrive/f1tenth_1/lidar";
  static constexpr const char* SPEED_TOPIC = "autodrive/f1tenth_1/speed";
  static constexpr const char* HEADING_TOPIC = "autodrive/f1tenth_1/imu";
	static constexpr const char* THROTTLE_CMD_TOPIC = "/autodrive/f1tenth_1/throttle_command";
	static constexpr const char* THROTTLE_CMD_RAW_TOPIC = "/autodrive/f1tenth_1/throttle_command_raw";

	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr heading_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_subscription_; 
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr throttle_cmd_raw_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr aeb_publisher_;
	
  void perform_aeb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
	void ingest_heading(const sensor_msgs::msg::Imu::SharedPtr msg);
	void ingest_speed(const std_msgs::msg::Float32::SharedPtr msg);
	void ingest_throttle_cmd_raw(const std_msgs::msg::Float32::SharedPtr msg);
	bool is_within_threshold();
	float convert_to_yaw(const sensor_msgs::msg::Imu::SharedPtr msg);
};

}

#endif


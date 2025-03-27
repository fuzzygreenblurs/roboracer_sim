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
private:
	float speed_;
	float heading_;
  float throttle_cmd_raw_;
  float min_ttc_ = 100; 
  float min_range_;
  bool seed_scan_ = false;
  bool seed_speed_ = false;
  bool seed_heading_ = false;
  bool enable_aeb_ = true;  
  std::vector<float> ttc_map_; 
  std::vector<float> filtered_ranges_; 
  std::vector<float> filtered_angles_; 
  std::vector<float> filtered_range_rates_; 
//	Eigen::ArrayXf ttc_map_;	

	static constexpr double EPSILON = 0.0001f;	
	static constexpr size_t BEAM_COUNT = 1080;
	static constexpr float TTC_THRESHOLD = 0.4;
	static constexpr float RANGE_THRESHOLD = 1;
	static constexpr float SPEED_THRESHOLD = 0.01;
  static constexpr float FOV_LIMIT = M_PI / 12.0f;

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
  rclcpp::TimerBase::SharedPtr aeb_timer_;

  void perform_aeb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
	void ingest_heading(const sensor_msgs::msg::Imu::SharedPtr msg);
	void ingest_speed(const std_msgs::msg::Float32::SharedPtr msg);
	void ingest_throttle_cmd_raw(const std_msgs::msg::Float32::SharedPtr msg);
  void brake_enforcer();
	float convert_to_yaw(const sensor_msgs::msg::Imu::SharedPtr msg);
};

}

#endif


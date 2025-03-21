#include <chrono>
#include "aeb/safety_node.hpp"

using std::placeholders::_1;

namespace AEB {
	SafetyNode::SafetyNode() : rclcpp::Node("aeb_safety_node") {
    enable_aeb_ = false;	

    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
			SCAN_TOPIC, 10, 
			std::bind(&SafetyNode::perform_aeb, this, _1)
		);

 	  heading_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
			HEADING_TOPIC, 10,
			std::bind(&SafetyNode::ingest_heading, this, _1)
		);

		speed_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
			SPEED_TOPIC, 10,
			std::bind(&SafetyNode::ingest_speed, this, _1)
		);

		throttle_cmd_raw_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
			THROTTLE_CMD_RAW_TOPIC, 10,
			std::bind(&SafetyNode::ingest_throttle_cmd_raw, this, _1)
		);

		aeb_publisher_ = this->create_publisher<std_msgs::msg::Float32>(THROTTLE_CMD_TOPIC, 10);

		auto timer_ = this->create_wall_timer(
			std::chrono::milliseconds(5),
			std::bind(&SafetyNode::brake, this)
		);
	} 

	//void SafetyNode::ingest_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
	void SafetyNode::perform_aeb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    enable_aeb_ = false;

		Eigen::Map<const Eigen::VectorXf> ranges(
			msg->ranges.data(),
			msg->ranges.size()
		);

		Eigen::VectorXf relative_angles = Eigen::VectorXf::LinSpaced(
			msg->ranges.size(),	
			msg->angle_min,
			msg->angle_min + (msg->angle_increment * (msg->ranges.size() - 1)) - heading_ 
		);
		
		Eigen::ArrayXf range_rates = speed_ * relative_angles.array().cos(); 
		
		ttc_map_ = ranges.array() / (-range_rates.max(0.0f) + EPSILON);

    if(is_within_threshold()) {
      enable_aeb_ = true; 
      RCLCPP_INFO(this->get_logger(), "checking if within threshold...");
      brake();
    }
	}

	void SafetyNode::ingest_speed(const std_msgs::msg::Float32::SharedPtr msg) {
		speed_ = msg->data; 
		RCLCPP_INFO(this->get_logger(), "longitudinal_velocity: %f", speed_); 
	}

  void SafetyNode::ingest_throttle_cmd_raw(const std_msgs::msg::Float32::SharedPtr msg) {
    if(enable_aeb_){
      RCLCPP_INFO(this->get_logger(), "AEB::CriticalThresholdDetected : trigger brake from raw cmd");
      return;
    }

    aeb_publisher_->publish(*msg); 
  }

  void SafetyNode::ingest_heading(const sensor_msgs::msg::Imu::SharedPtr msg) {
    float qw = msg->orientation.w;
    float qx = msg->orientation.x;
    float qy = msg->orientation.y;
    float qz = msg->orientation.z;

      
    float yaw = std::atan2(
      2.0 * ((qw * qx) + (qy * qz)),
      1.0 - (2.0 * (qx * qx) + (qy * qy))
    );

    heading_ = yaw;
  }

	bool SafetyNode::is_within_threshold() {
    RCLCPP_INFO(this->get_logger(), "min TTC value: %f", ttc_map_.minCoeff());
		return ttc_map_.minCoeff() <= AEB_MIN_RADIUS; 
	}

	void SafetyNode::brake() {
    std_msgs::msg::Float32 msg;
    msg.data = 0.0f;
    aeb_publisher_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "AEB::CriticalThresholdDetected : trigger brake");
    return; 	
  }
};

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<AEB::SafetyNode>());
	rclcpp::shutdown();

	return 0;
}

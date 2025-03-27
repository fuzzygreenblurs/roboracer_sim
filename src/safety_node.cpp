#include <chrono>
#include "aeb/safety_node.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace AEB {
	SafetyNode::SafetyNode() : rclcpp::Node("aeb_safety_node") {
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

    RCLCPP_INFO(this->get_logger(), "waiting for initial payloads...");
    while(!seed_scan_ && !seed_speed_ && rclcpp::ok()) {
      /*
        - process any pending callbacks
        - do not block: check callback queue once and return immediately
        - allow the subscription callback to fire if msg arrives
      */
      rclcpp::spin_some(this->get_node_base_interface()); 
    }

    RCLCPP_INFO(this->get_logger(), "initial payloads received... continuing initialization");
    rclcpp::sleep_for(5s);

		throttle_cmd_raw_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
			THROTTLE_CMD_RAW_TOPIC, 10,
			std::bind(&SafetyNode::ingest_throttle_cmd_raw, this, _1)
		);

		aeb_publisher_ = this->create_publisher<std_msgs::msg::Float32>(THROTTLE_CMD_TOPIC, 10);

    aeb_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(5),
      std::bind(&SafetyNode::brake_enforcer, this)
    );
  } 

  void SafetyNode::brake_enforcer() {
    if(enable_aeb_) {
	  	RCLCPP_INFO(this->get_logger(), "min_range %.3f", min_range_); 
      if(speed_ >= SPEED_THRESHOLD || min_ttc_ <= TTC_THRESHOLD) {
	  	  RCLCPP_INFO(this->get_logger(), "ENABLED: COOLING DOWN"); 
        std_msgs::msg::Float32 msg;
        msg.data = 0.0f;
        aeb_publisher_->publish(msg);
      } else {
        RCLCPP_INFO(this->get_logger(), "DISABLED: drive.:"); 
        enable_aeb_ = false;
      }
    }
  }

  void SafetyNode::perform_aeb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if(!seed_scan_) {
       seed_scan_ = true;
    }

    for(size_t i = 0; i < msg->ranges.size(); ++i) {
      float angle = msg->angle_min + (i * msg->angle_increment);
      float range = msg->ranges[i];

      if(std::abs(angle) > FOV_LIMIT || !std::isfinite(range) || range <= 0.0f) {
        continue;
      }
  
      filtered_ranges_.push_back(range);
      filtered_angles_.push_back(angle);

      float range_rate = speed_ * std::cos(angle);

      if(range_rate <= EPSILON) {
        continue;
      }

      filtered_range_rates_.push_back(range_rate);

      float ttc = range / range_rate;

      ttc_map_.push_back(ttc);


      if(ttc > 0.0f && ttc < min_ttc_) {
        min_ttc_ = ttc;
      }

      if(range < min_range_) {
        min_range_ = range;
      }
    }

    if(min_ttc_ <= TTC_THRESHOLD) {
      enable_aeb_ = true;
    }
  }

//void SafetyNode::perform_aeb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
//  const float FOV_LIMIT = M_PI / 12.0f;
//  int num_valid = 0;
//  for (int i = 0; i < msg->ranges.size(); i++) {
//    float angle = msg->angle_min + (i* msg->angle_increment);
//    float r = msg->ranges[i];

//    if(std::abs(angle) <= FOV_LIMIT && std::isfinite(r) && r > 0.0f) {
//      num_valid++;
//    }
//  }
//  
//  if(num_valid == 0) {
//    RCLCPP_WARN(this->get_logger(), "AEB::No valid beams in FOV");
//    return;
//  }

//  Eigen::VectorXf ranges(num_valid);
//  Eigen::VectorXf angles(num_valid);

//  int j = 0;
//  for (int i = 0; i < msg->ranges.size(); i++) {
//    float angle = msg->angle_min + (i* msg->angle_increment);
//    float r = msg->ranges[i];

//    if(std::abs(angle) <= FOV_LIMIT && std::isfinite(r) && r > 0.0f) {
//      ranges[j] = r;
//      angles[j] = angle; 
//      j++;
//    }
//  }

//  Eigen::ArrayXf range_rates = speed_ * angles.array().cos();
//  ttc_map_ = ranges.array() / (range_rates + EPSILON);

//  for(int i = 0; i < ttc_map_.size(); i++) {
//    if(ttc_map_[i] < 0 || !std::isfinite(ttc_map_[i])) {
//      ttc_map_[i] = std::numeric_limits<float>::infinity();
//    }
//  }

//  ttc_map_min_ = ttc_map_.minCoeff();
//  if(!enable_aeb_ && is_within_threshold()){
//    RCLCPP_INFO(this->get_logger(), "ENABLED: within_threshold");
//    enable_aeb_ = true; 
//  }
//}

	void SafetyNode::ingest_speed(const std_msgs::msg::Float32::SharedPtr msg) {
    if(!seed_speed_) {
      seed_speed_ = true;
    }

		speed_ = msg->data; 
	}

  void SafetyNode::ingest_throttle_cmd_raw(const std_msgs::msg::Float32::SharedPtr msg) {
    if(enable_aeb_) {
      RCLCPP_INFO(this->get_logger(), "ENABLED: ignore incoming");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "DISABLED: forward throttle ---> ");
    aeb_publisher_->publish(*msg); 
  }

  void SafetyNode::ingest_heading(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if(!seed_heading_) {
      seed_heading_ = true; 
    }

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
};

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<AEB::SafetyNode>());
	rclcpp::shutdown();

	return 0;
}

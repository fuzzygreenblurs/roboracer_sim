#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using std::placeholders::_1;

class AEBTest : public rclcpp::Node {
public:
  AEBTest() : Node("AEB::Test") {
    throttle_ = 0.3f;
    throttle_pub_ = rclcpp::Publisher<std_msgs::msg::Float32>("/throttle_command_raw", rclcpp::QoS(10));
    timer_ = create_wall_timer(100ms, std::bind(&AEBTest::publish_throttle, this)); 
    RCLCPP_INFO(get_logger(), "AEB::Test node started iwht throttle: %.2f", throttle_); 
  }

  

private:
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  float throttle_;

  void publish_throttle() {
    auto msg = std_msgs::msg::Float32{};
    msg.data = throttle_;
    throttle_pub_->publish(msg);
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AEBTest>());
  rclcpp::shutdown();
  return 0;
}

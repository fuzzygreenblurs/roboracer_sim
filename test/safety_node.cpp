#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using std::placeholders::_1;

class AEBTest : public rclcpp::Node {
public:
  AEBTest() : Node("SafetyNodeTest") {
    this->declare_parameter("test_throttle", 1.0f);
    throttle_ = this->get_parameter("test_throttle").as_double();
    throttle_pub_ = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/throttle_command_raw", rclcpp::QoS(10));
    timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&AEBTest::publish_throttle, this)); 
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
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AEBTest>());
  rclcpp::shutdown();
  return 0;
}

#include "rclcpp/rclcpp.hpp"
#include "drivers/msg/input.hpp"

using std::placeholders::_1;

class ManualControl : public rclcpp::Node {
 public:
  ManualControl() : Node("manual_control") {
    subscriber = this->create_subscription<drivers::msg::Input>("xbox_controller_input", 1, std::bind(&ManualControl::callback, this, _1));
  }
 private:
  rclcpp::Subscription<drivers::msg::Input>::SharedPtr subscriber;
  
  void callback(const drivers::msg::Input::SharedPtr input) const {
    RCLCPP_INFO(this->get_logger(), "left joystick = %d", input->left_stick_x);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManualControl>());
  rclcpp::shutdown();
  return 0;
}

#include <linux/input.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "drivers/msg/input.hpp"

namespace drivers {

class XboxController : public rclcpp::Node {
 public:
  XboxController(const rclcpp::NodeOptions& options);
 
 private:
  rclcpp::Publisher<drivers::msg::Input>::SharedPtr publisher;
  rclcpp::TimerBase::SharedPtr timer;
  drivers::msg::Input input;
  std::string device;
  int fd;

  void callback();
  void input_map(struct input_event event);
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(drivers::XboxController)

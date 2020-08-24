#include <fcntl.h>
#include <unistd.h>

#include "xbox_controller.hpp"

namespace drivers {

using namespace std::chrono_literals;

XboxController::XboxController(const rclcpp::NodeOptions& options) : Node("xbox_controller", options) {
  this->declare_parameter<std::string>("device", "/dev/input/event3");

  RCLCPP_INFO(this->get_logger(), "Reading parameters");
  this->get_parameter("device", device);

  RCLCPP_INFO(this->get_logger(), "Opening %s", device.c_str());
  fd = open(device.c_str(), O_RDONLY | O_NONBLOCK);

  if (fd == -1) {
    RCLCPP_ERROR(this->get_logger(), "Error opening device");
  } else {
    RCLCPP_INFO(this->get_logger(), "Starting node");
    publisher = this->create_publisher<drivers::msg::Input>("xbox_controller_input", 1);
    timer = this->create_wall_timer(1ms, std::bind(&XboxController::callback, this));
  }
}
 
void XboxController::callback() {
  struct input_event event;

  if (read(fd, &event, sizeof(event)) == sizeof(event)) {
    input_map(event);
  }

  publisher->publish(input);
}
  
void XboxController::input_map(struct input_event event) {
  if (event.type == EV_KEY) {
    switch (event.code) {
      case BTN_A:
        input.a = event.value;
        break;
      case BTN_B:
        input.b = event.value;
        break;
      case BTN_X:
        input.x = event.value;
        break;
      case BTN_Y:
        input.y = event.value;
        break;
      case BTN_THUMBL:
        input.left_joystick = event.value;
        break;
      case BTN_TL:
        input.left_bumper = event.value;
        break;
      case BTN_THUMBR:
        input.right_joystick = event.value;
        break;
      case BTN_TR:
        input.right_bumper = event.value;
        break;
      case BTN_START:
        input.start = event.value;
        break;
      case KEY_BACK:
        input.select = event.value;
        break;
      case BTN_MODE:
        input.home = event.value;
        break;
    }
  } else if (event.type == EV_ABS) {
    switch (event.code) {
      case ABS_X:
        input.left_joystick_x = event.value;
        break;
      case ABS_Y:
        input.left_joystick_y = event.value;
        break;
      case ABS_BRAKE:
        input.left_trigger = event.value;
        break;
      case ABS_Z:
        input.right_joystick_x = event.value;
        break;
      case ABS_RZ:
        input.right_joystick_y = event.value;
        break;
      case ABS_GAS:
        input.right_trigger = event.value;
        break;
      case ABS_HAT0Y:
        input.up = event.value < 0 ? true : false;
        input.down = event.value > 0 ? true : false;
        break;
      case ABS_HAT0X:
        input.left = event.value < 0 ? true : false;
        input.right = event.value > 0 ? true : false;
        break;
    }
  }
}

}


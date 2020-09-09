#include "formula_a/components/ManualControl.hpp"

namespace isaac {

void ManualControl::start() {
  tickPeriodically();
}

void ManualControl::tick() {
  if (rx_gamepad().available()) {
    auto gamepad = rx_gamepad().getProto();
    //auto buttons = gamepad.getButtons();
    auto axes = gamepad.getAxes();

    double steering = axes[0].getX();
    double throttle = axes[2].getX();
    double brake = axes[2].getY();

    LOG_INFO("S = %0.2f | T = %0.2f | B = %0.2f", steering, throttle, brake);
    
    /*
    for (auto button : buttons) {
      LOG_INFO("%d", button);
    }
    */
    
    LOG_INFO("-----");

  }
}

void ManualControl::stop() {

}

}  // namespace isaac

#pragma once

#include "engine/alice/alice_codelet.hpp"
#include "messages/joystick_state.capnp.h"

namespace isaac {

class ManualControl : public alice::Codelet {
  void start() override;
  void tick() override;
  void stop() override;

  ISAAC_PROTO_RX(JoystickStateProto, gamepad);
};

}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::ManualControl);

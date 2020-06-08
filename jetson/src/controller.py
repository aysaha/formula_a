import os
import time
import threading
import argparse

from evdev import InputDevice, list_devices, ecodes
from serial import Serial

FILE = os.path.basename(__file__)
DIRECTORY = os.path.dirname(__file__)

class XboxController:
    MAX_JOYSTICK = 65535
    MAX_TRIGGER = 1023

    def __init__(self):
        assert len(list_devices()) == 1

        self.device = InputDevice(list_devices()[0])
        self.thread = threading.Thread(target=self.monitor)
        self.A = 0
        self.B = 0
        self.X = 0
        self.Y = 0
        self.LX = XboxController.MAX_JOYSTICK // 2 + 1
        self.LY = XboxController.MAX_JOYSTICK // 2 + 1
        self.LS = 0
        self.LB = 0
        self.LT = 0
        self.RX = XboxController.MAX_JOYSTICK // 2 + 1
        self.RY = XboxController.MAX_JOYSTICK // 2 + 1
        self.RS = 0
        self.RB = 0
        self.RT = 0
        self.UP = 0
        self.DOWN = 0
        self.LEFT = 0
        self.RIGHT = 0
        self.HOME = 0
        self.START = 0
        self.SELECT = 0

        print("[{}] starting daemon thread".format(FILE))
        self.thread.daemon = True
        self.thread.start()

    def monitor(self):
        print("[{}] daemon thread started".format(FILE))
        for event in self.device.read_loop():
            if event.type == ecodes.EV_KEY:
                if event.code == ecodes.BTN_A:          self.A = event.value
                elif event.code == ecodes.BTN_B:        self.B = event.value
                elif event.code == ecodes.BTN_X:        self.X = event.value
                elif event.code == ecodes.BTN_Y:        self.Y = event.value
                elif event.code == ecodes.BTN_THUMBL:   self.LS = event.value
                elif event.code == ecodes.BTN_THUMBR:   self.RS = event.value
                elif event.code == ecodes.BTN_TL:       self.LB = event.value
                elif event.code == ecodes.BTN_TR:       self.RB = event.value
                elif event.code == ecodes.BTN_MODE:     self.HOME = event.value
                elif event.code == ecodes.BTN_START:    self.START = event.value
                elif event.code == ecodes.KEY_BACK:     self.SELECT = event.value
            elif event.type == ecodes.EV_ABS:
                if event.code == ecodes.ABS_X:          self.LX = event.value
                elif event.code == ecodes.ABS_Y:        self.LY = event.value
                elif event.code == ecodes.ABS_Z:        self.RX = event.value
                elif event.code == ecodes.ABS_RZ:       self.RY = event.value
                elif event.code == ecodes.ABS_BRAKE:    self.LT = event.value
                elif event.code == ecodes.ABS_GAS:      self.RT = event.value
                elif event.code == ecodes.ABS_HAT0X:
                    if event.value < 0:
                        self.LEFT = event.value * -1
                        self.RIGHT = 0
                    elif event.value > 0:
                        self.LEFT = 0
                        self.RIGHT = event.value
                    else:
                        self.LEFT = 0
                        self.RIGHT = 0
                elif event.code == ecodes.ABS_HAT0Y:
                    if event.value < 0:
                        self.UP = event.value * -1
                        self.DOWN = 0
                    elif event.value > 0:
                        self.UP = 0
                        self.DOWN = event.value
                    else:
                        self.UP = 0
                        self.DOWN = 0


    def read(self):
        steer = ((self.LX / XboxController.MAX_JOYSTICK * 2) - 1) ** 3
        accel = (self.RT / XboxController.MAX_TRIGGER) ** 2
        brake = (self.LT / XboxController.MAX_TRIGGER) ** 2
        done = True if self.B == 1 else False

        return steer, accel, brake, done

def main(args):
    assert 1 <= args.power <= 100
    assert os.path.exists(args.port)

    # initialize controller
    controller = XboxController()
    done = False

    # initialize connection
    connection = Serial(args.port, 115200)
    time.sleep(1)

    print("[{}] starting test".format(FILE))
    print("\n  STEER | ACCEL | BRAKE || CH 1 | CH 2")
    while not done:
        # read controller input
        steer, accel, brake, done = controller.read()

        # limit throttle
        accel *= args.power / 100

        # convert inputs to servo commands
        position = int(500 * steer + 1500)
        speed = int(500 * (accel - brake) + 1500)

        # transmit commands
        data = bytes([(position >> 8) & 0xFF, position & 0xFF, (speed >> 8) & 0xFF, speed & 0xFF])
        connection.write(data)

        # display inputs and commands
        print("  {:5.2f} | {:5.2f} | {:5.2f} || {:4d} | {:4d}".format(steer, accel, brake, position, speed), end='\r')
        time.sleep(0.1)
    print("\n")
    print("[{}] stopping test".format(FILE))

if __name__ == '__main__':
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('-p', '--power', metavar='power', type=int, default=100)
    parser.add_argument('port')
    args = parser.parse_args()
    main(args)

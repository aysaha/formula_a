import os
import time
import multiprocessing
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
        self.A = multiprocessing.Value('i', 0)
        self.B = multiprocessing.Value('i', 0)
        self.X = multiprocessing.Value('i', 0)
        self.Y = multiprocessing.Value('i', 0)
        self.LX = multiprocessing.Value('i', 0)
        self.LY = multiprocessing.Value('i', 0)
        self.LS = multiprocessing.Value('i', 0)
        self.LB = multiprocessing.Value('i', 0)
        self.LT = multiprocessing.Value('i', 0)
        self.RX = multiprocessing.Value('i', 0)
        self.RY = multiprocessing.Value('i', 0)
        self.RS = multiprocessing.Value('i', 0)
        self.RB = multiprocessing.Value('i', 0)
        self.RT = multiprocessing.Value('i', 0)
        self.HOME = multiprocessing.Value('i', 0)
        self.START = multiprocessing.Value('i', 0)
        self.SELECT = multiprocessing.Value('i', 0)
        self.process = multiprocessing.Process(target=self.monitor_controller)

        print("[{}] starting daemon process".format(FILE))
        self.process.daemon = True
        self.process.start()
        time.sleep(1)

    def monitor_controller(self):
        print("[{}] daemon process started".format(FILE))

        for event in self.device.read_loop():
            if event.type == ecodes.EV_KEY:
                if event.code == ecodes.BTN_A:          self.A.value = event.value
                elif event.code == ecodes.BTN_B:        self.B.value = event.value
                elif event.code == ecodes.BTN_X:        self.X.value = event.value
                elif event.code == ecodes.BTN_Y:        self.Y.value = event.value
                elif event.code == ecodes.BTN_THUMBL:   self.LS.value = event.value
                elif event.code == ecodes.BTN_TL:       self.LB.value = event.value
                elif event.code == ecodes.BTN_THUMBR:   self.RS.value = event.value
                elif event.code == ecodes.BTN_TR:       self.RB.value = event.value
                elif event.code == ecodes.BTN_MODE:     self.HOME.value = event.value
                elif event.code == ecodes.BTN_START:    self.START.value = event.value
                elif event.code == ecodes.KEY_BACK:     self.SELECT.value = event.value
            elif event.type == ecodes.EV_ABS:
                if event.code == ecodes.ABS_X:          self.LX.value = event.value
                elif event.code == ecodes.ABS_Y:        self.LY.value = event.value
                elif event.code == ecodes.ABS_BRAKE:    self.LT.value = event.value
                elif event.code == ecodes.ABS_Z:        self.RX.value = event.value
                elif event.code == ecodes.ABS_RZ:       self.RY.value = event.value
                elif event.code == ecodes.ABS_GAS:      self.RT.value = event.value

    def read(self):
        steer = ((self.LX.value / XboxController.MAX_JOYSTICK * 2) - 1) ** 3
        accel = (self.RT.value / XboxController.MAX_TRIGGER) ** 2
        brake = (self.LT.value / XboxController.MAX_TRIGGER) ** 2
        done = True if self.B.value == 1 else False

        return [steer, accel, brake], done

def main(args):
    controller = XboxController()
    done = False

    arduino = Serial(args.port, 115200)
    time.sleep(1)

    print("[{}] running test".format(FILE))
    print("\n  STEER | ACCEL | BRAKE || CH 1 | CH 2")
    while not done:
        controller.read()
        action, done = controller.read()
        steer, accel, brake = action
        
        position = int(500 * steer + 1500)
        speed = int(500 * (accel - brake) + 1500)

        print("  {:5.2f} | {:5.2f} | {:5.2f} || {:4d} | {:4d}".format(steer, accel, brake, position, speed), end='\r')
        
        data = bytes([(position >> 8) & 0xFF, position & 0xFF, (speed >> 8) & 0xFF, speed & 0xFF])
        arduino.write(data)
        
        time.sleep(0.1)
    print("\n")
    print("[{}] stopping test".format(FILE))

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('port', help='serial port connected to arduino')
    args = parser.parse_args()
    main(args)

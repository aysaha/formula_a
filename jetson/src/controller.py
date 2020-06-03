#!/usr/bin/env python

import os
import time
import multiprocessing
import argparse

import numpy as np
import inputs
import serial

FILE = os.path.basename(__file__)
DIRECTORY = os.path.dirname(__file__)

class XboxController:
    MAX_JOY_VAL = np.power(2, 15) - 1
    MAX_TRIG_VAL = np.power(2, 8) - 1

    def __init__(self):
        assert len(inputs.devices.gamepads) == 1

        self.LeftJoystickX = multiprocessing.Value('i', 0)
        self.LeftJoystickY = multiprocessing.Value('i', 0)
        self.RightJoystickX = multiprocessing.Value('i', 0)
        self.RightJoystickY = multiprocessing.Value('i', 0)
        self.LeftTrigger = multiprocessing.Value('i', 0)
        self.RightTrigger = multiprocessing.Value('i', 0)
        self.LeftBumper = multiprocessing.Value('i', 0)
        self.RightBumper = multiprocessing.Value('i', 0)
        self.A = multiprocessing.Value('i', 0)
        self.B = multiprocessing.Value('i', 0)
        self.X = multiprocessing.Value('i', 0)
        self.Y = multiprocessing.Value('i', 0)
        self.LeftThumb = multiprocessing.Value('i', 0)
        self.RightThumb = multiprocessing.Value('i', 0)
        self.Back = multiprocessing.Value('i', 0)
        self.Start = multiprocessing.Value('i', 0)
        self.LeftDPad = multiprocessing.Value('i', 0)
        self.RightDPad = multiprocessing.Value('i', 0)
        self.UpDPad = multiprocessing.Value('i', 0)
        self.DownDPad = multiprocessing.Value('i', 0)
        self.process = multiprocessing.Process(target=self.monitor_controller)

        print("[{}] starting daemon process".format(FILE))
        self.process.daemon = True
        self.process.start()
        time.sleep(1)

    def monitor_controller(self):
        print("[{}] daemon process started".format(FILE))
        while True:
            time.sleep(0.001)
            events = inputs.get_gamepad()

            for event in events:
                if event.code == 'ABS_X': self.LeftJoystickX.value = event.state
                elif event.code == 'ABS_Y': self.LeftJoystickY.value = event.state
                elif event.code == 'ABS_RX': self.RightJoystickX.value = event.state
                elif event.code == 'ABS_RY': self.RightJoystickY.value = event.state
                elif event.code == 'ABS_Z': self.LeftTrigger.value = event.state
                elif event.code == 'ABS_RZ': self.RightTrigger.value = event.state
                elif event.code == 'BTN_TL': self.LeftBumper.value = event.state
                elif event.code == 'BTN_TR': self.RightBumper.value = event.state
                elif event.code == 'BTN_SOUTH': self.A.value = event.state
                elif event.code == 'BTN_EAST': self.B.value = event.state
                elif event.code == 'BTN_NORTH': self.X.value = event.state
                elif event.code == 'BTN_WEST': self.Y.value = event.state
                elif event.code == 'BTN_THUMBL': self.LeftThumb.value = event.state
                elif event.code == 'BTN_THUMBR': self.RightThumb.value = event.state
                elif event.code == 'BTN_SELECT': self.Back.value = event.state
                elif event.code == 'BTN_START': self.Start.value = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY1': self.LeftDPad.value = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY2': self.RightDPad.value = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY3': self.UpDPad.value = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY4': self.DownDPad.value = event.state

    def read(self):
        steer = self.LeftJoystickX.value / XboxController.MAX_JOY_VAL
        accel = self.RightTrigger.value / XboxController.MAX_TRIG_VAL
        brake = self.LeftTrigger.value / XboxController.MAX_TRIG_VAL

        action = np.clip(np.power([steer, accel, brake], [3, 2, 2]), [-1, 0, 0], [1, 1, 1])
        done = True if self.B.value == 1 else False

        return action, done

def main(args):
    controller = XboxController()
    done = False

    arduino = serial.Serial(args.port, 115200)
    time.sleep(1)

    print("[{}] running test".format(FILE))
    print("\n  STEER | ACCEL | BRAKE || CH 1 | CH 2")
    while not done:
        action, done = controller.read()
        steer, accel, brake = action

        position = int(500 * steer + 1500)
        speed = int(500 * (accel - brake) + 1500)

        print("  {:5.2f} | {:5.2f} | {:5.2f} || {:4d} | {:4d}".format(steer, accel, brake, position, speed), end='\r')

        buffer = bytes([(position >> 8) & 0xFF, position & 0xFF, (speed >> 8) & 0xFF, speed & 0xFF])
        arduino.write(buffer)

        time.sleep(0.1)
    print("\n")
    print("[{}] stopping test".format(FILE))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('port', help='serial port connected to arduino')
    args = parser.parse_args()
    main(args)

#!/usr/bin/python3

import os
import time
import argparse

import numpy as np
import cv2
import pyrealsense2 as rs

FILE = os.path.basename(__file__)
DIRECTORY = os.path.dirname(__file__)

class DepthCamera:
    def __init__(self, width=640, height=480, framerate=30):
        self.pipeline = rs.pipeline()

        config = rs.config()
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, framerate)
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, framerate)
        config.enable_stream(rs.stream.accel)
        config.enable_stream(rs.stream.gyro)
        self.pipeline.start(config)

    def __del__(self):
        self.pipeline.stop()

    def read(self):
        frames = self.pipeline.wait_for_frames()
        depth_data = frames[0].get_data()
        color_data = frames[1].get_data()
        accel_data = frames[2].as_motion_frame().get_motion_data()
        gyro_data = frames[3].as_motion_frame().get_motion_data()

        depth = np.asanyarray(depth_data)
        color = np.asanyarray(color_data)
        accel = np.array([accel_data.x, accel_data.y, accel_data.z])
        gyro = np.array([gyro_data.x, gyro_data.y, gyro_data.z])

        return depth, color, accel, gyro

def main(args):
    camera = DepthCamera()

    print("[{}] starting test".format(FILE))
    print("\n      ACCEL (x, y, z)   |     GYRO (x, y, z)")
    while True:
        try:
            depth, color, accel, gyro = camera.read()
            print("  ({:5.2f}, {:5.2f}, {:5.2f}) | ({:5.2f}, {:5.2f}, {:5.2f})".format(accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]), end='\r')
            #time.sleep(0.1)

            colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET)
            image = np.hstack((color, colormap))

            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', image)
            cv2.waitKey(1)
        except KeyboardInterrupt:
            break
    print("\n")
    print("[{}] stopping test".format(FILE))

    cv2.destroyAllWindows()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(add_help=False)
    args = parser.parse_args()
    main(args)

# File: /raspberry-pi-robot/raspberry-pi-robot/src/sensors/lidar.py

import RPi.GPIO as GPIO
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from rplidar import RPLidar

class Lidar:
    def __init__(self, port='/dev/ttyUSB0'):
        self.lidar = RPLidar(port)
        self.scan_data = []
        self.running = False

    def start(self):
        self.running = True
        print("Starting LiDAR...")
        for scan in self.lidar.scan():
            if not self.running:
                break
            self.process_scan(scan)

    def stop(self):
        self.running = False
        self.lidar.stop()
        self.lidar.disconnect()
        print("LiDAR stopped.")

    def process_scan(self, scan):
        angle, distance = scan[1], scan[2]
        if distance < 4000:  # Ignore outliers
            self.scan_data.append((angle, distance))
        self.visualize_scan()

    def visualize_scan(self):
        if not self.scan_data:
            return
        angles, distances = zip(*self.scan_data)
        plt.clf()
        plt.polar(np.radians(angles), distances, 'r.')
        plt.title("LiDAR Scan Data")
        plt.pause(0.1)

    def get_obstacles(self, threshold=500):
        obstacles = [(angle, distance) for angle, distance in self.scan_data if distance < threshold]
        return obstacles

if __name__ == "__main__":
    lidar = Lidar()
    try:
        lidar.start()
    except KeyboardInterrupt:
        lidar.stop()
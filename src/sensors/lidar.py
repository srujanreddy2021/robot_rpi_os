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
        
        # Initialize the plot in polar coordinates
        plt.figure(figsize=(8, 8))
        self.ax = plt.subplot(111, projection='polar')
        self.ax.set_title('LIDAR Scan')
        self.line, = self.ax.plot([], [], 'r.', markersize=2)
        self.ax.set_rmax(5000)  # Adjust based on your LIDAR's range
        self.ax.grid(True)
        
        # Get and print LIDAR info and health
        info = self.lidar.get_info()
        print(f"LIDAR Info: {info}")
        health = self.lidar.get_health()
        print(f"LIDAR Health: {health}")

    def start(self):
        self.running = True
        print("Starting LiDAR...")
        try:
            for i, scan in enumerate(self.lidar.iter_scans()):
                if not self.running:
                    break
                print(f"{i}: Got {len(scan)} measurements")
                self.process_scan(scan)
        except Exception as e:
            print(f"Error in LIDAR scanning: {e}")
            self.stop()

    def stop(self):
        self.running = False
        print("Stopping LiDAR...")
        try:
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()
            print("LiDAR stopped successfully.")
        except Exception as e:
            print(f"Error stopping LIDAR: {e}")

    def process_scan(self, scan):
        # Process each measurement in the scan
        if not scan:
            return
            
        # Extract angles (in radians) and distances
        angles = np.array([measurement[1] * np.pi / 180 for measurement in scan])
        distances = np.array([measurement[2] for measurement in scan])
        
        # Filter out outliers
        mask = distances < 4000
        self.scan_data = list(zip(
            np.degrees(angles[mask]),  # Convert back to degrees for storage
            distances[mask]
        ))
        
        # Visualize the scan
        self.visualize_scan(angles[mask], distances[mask])

    def visualize_scan(self, angles=None, distances=None):
        if angles is None or distances is None:
            # If no direct data provided, extract from scan_data
            if not self.scan_data:
                return
            angles_deg, distances = zip(*self.scan_data)
            angles = np.radians(angles_deg)  # Convert to radians for plotting
        
        # Update plot data
        self.line.set_xdata(angles)
        self.line.set_ydata(distances)
        
        # Redraw the plot
        plt.draw()
        plt.pause(0.001)  # Small pause for plot to update

    def get_obstacles(self, threshold=500):
        obstacles = [(angle, distance) for angle, distance in self.scan_data if distance < threshold]
        return obstacles

if __name__ == "__main__":
    lidar = Lidar()
    try:
        lidar.start()
    except KeyboardInterrupt:
        print("\nLiDAR scanning stopped by user.")
    finally:
        lidar.stop()
        plt.show()  # Keep the plot open after stopping
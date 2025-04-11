# slam.py

import numpy as np
import matplotlib.pyplot as plt
from sensors.lidar import RPLidar
from sensors.imu import MPU6050

class SLAM:
    def __init__(self):
        self.lidar = RPLidar()
        self.imu = MPU6050()
        self.position = np.array([0.0, 0.0])  # Robot's position (x, y)
        self.orientation = 0.0  # Robot's orientation in degrees
        self.map = []  # List to hold map points

    def update_position(self):
        # Get IMU data
        accel_data = self.imu.get_acceleration()
        gyro_data = self.imu.get_gyroscope()
        
        # Apply Kalman filter or complementary filter here if needed
        self.orientation += gyro_data[2]  # Update orientation based on gyroscope
        self.orientation = self.orientation % 360  # Normalize to [0, 360)

        # Estimate translation based on acceleration
        translation = np.array([
            accel_data[0] * np.cos(np.radians(self.orientation)),
            accel_data[1] * np.sin(np.radians(self.orientation))
        ])
        
        self.position += translation  # Update position

    def update_map(self):
        # Get LiDAR data
        lidar_data = self.lidar.get_scan()
        for angle, distance in lidar_data:
            # Convert polar coordinates to Cartesian coordinates
            x = distance * np.cos(np.radians(angle))
            y = distance * np.sin(np.radians(angle))
            self.map.append((x, y))

    def visualize(self):
        plt.clf()
        # Plot the map
        if self.map:
            x_map, y_map = zip(*self.map)
            plt.scatter(x_map, y_map, c='blue', label='Map Points')

        # Plot the robot's position
        plt.scatter(self.position[0], self.position[1], c='red', label='Robot Position')
        plt.xlim(-10, 10)  # Adjust limits as necessary
        plt.ylim(-10, 10)
        plt.title('SLAM Visualization')
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.legend()
        plt.pause(0.1)

    def run(self):
        plt.ion()  # Interactive mode on
        while True:
            self.update_position()
            self.update_map()
            self.visualize()

if __name__ == "__main__":
    slam = SLAM()
    slam.run()
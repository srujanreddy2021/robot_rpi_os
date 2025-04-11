# File: /raspberry-pi-robot/raspberry-pi-robot/src/visualization/display.py

import pygame
import numpy as np
from sensors.lidar import Lidar
from sensors.imu import IMU

class RobotVisualizer:
    def __init__(self, lidar: Lidar, imu: IMU, width=800, height=600):
        pygame.init()
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Robot Visualization")
        self.lidar = lidar
        self.imu = imu
        self.clock = pygame.time.Clock()
        self.running = True

    def draw_lidar_data(self):
        lidar_data = self.lidar.get_data()
        for angle, distance in lidar_data:
            x = int(distance * np.cos(angle))
            y = int(distance * np.sin(angle))
            pygame.draw.line(self.screen, (255, 0, 0), (400, 300), (400 + x, 300 + y), 1)

    def draw_robot_position(self):
        position = self.imu.get_position()
        pygame.draw.circle(self.screen, (0, 255, 0), (400, 300), 10)  # Robot position

    def draw_obstacles(self):
        obstacles = self.lidar.detect_obstacles()
        for obstacle in obstacles:
            pygame.draw.circle(self.screen, (255, 255, 0), (obstacle[0], obstacle[1]), 5)

    def run(self):
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False

            self.screen.fill((0, 0, 0))  # Clear screen
            self.draw_lidar_data()
            self.draw_robot_position()
            self.draw_obstacles()
            pygame.display.flip()  # Update the display
            self.clock.tick(60)  # Limit to 60 FPS

        pygame.quit()

if __name__ == "__main__":
    lidar = Lidar()  # Initialize LiDAR
    imu = IMU()      # Initialize IMU
    visualizer = RobotVisualizer(lidar, imu)
    visualizer.run()
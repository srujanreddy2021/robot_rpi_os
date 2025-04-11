# File: /raspberry-pi-robot/raspberry-pi-robot/src/navigation/obstacle_avoidance.py

import numpy as np
from sensors.lidar import Lidar
from sensors.imu import IMU
from actuators.motors import Motors

class ObstacleAvoidance:
    def __init__(self, lidar: Lidar, imu: IMU, motors: Motors):
        self.lidar = lidar
        self.imu = imu
        self.motors = motors
        self.obstacle_threshold = 30  # cm, distance to consider as an obstacle

    def check_obstacles(self):
        distances = self.lidar.get_distance_data()
        for distance in distances:
            if distance < self.obstacle_threshold:
                return True
        return False

    def avoid_obstacle(self):
        if self.check_obstacles():
            self.motors.stop()  # Stop the robot
            print("Obstacle detected! Stopping.")
            # Implement a simple avoidance strategy
            self.motors.turn_right(90)  # Turn right to avoid the obstacle
            self.motors.move_forward(20)  # Move forward a bit
            self.motors.turn_left(90)  # Turn back to original direction
            print("Obstacle avoided, resuming path.")

    def navigate(self):
        while True:
            self.avoid_obstacle()
            self.motors.move_forward(10)  # Move forward continuously

# Example usage:
# lidar = Lidar()
# imu = IMU()
# motors = Motors()
# obstacle_avoidance = ObstacleAvoidance(lidar, imu, motors)
# obstacle_avoidance.navigate()
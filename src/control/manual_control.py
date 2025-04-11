# File: /raspberry-pi-robot/raspberry-pi-robot/src/control/manual_control.py

import pygame
import time
from actuators.motors import Motors
from sensors.lidar import Lidar

class ManualControl:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((800, 600))
        pygame.display.set_caption("Manual Control of Robot")
        self.clock = pygame.time.Clock()
        self.motors = Motors()
        self.lidar = Lidar()
        self.running = True

    def run(self):
        while self.running:
            self.handle_events()
            self.check_obstacles()
            self.update_display()
            self.clock.tick(60)

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    self.move_forward()
                elif event.key == pygame.K_DOWN:
                    self.move_backward()
                elif event.key == pygame.K_LEFT:
                    self.turn_left()
                elif event.key == pygame.K_RIGHT:
                    self.turn_right()

    def move_forward(self):
        self.motors.move_forward(1)  # Move forward by 10 cm

    def move_backward(self):
        self.motors.move_backward(1)  # Move backward by 10 cm

    def turn_left(self):
        self.motors.turn_left(5)  # Turn left by 15 degrees

    def turn_right(self):
        self.motors.turn_right(5)  # Turn right by 15 degrees

    def check_obstacles(self):
        if self.lidar.detect_obstacles():
            print("Obstacle detected! Please change direction.")

    def update_display(self):
        self.screen.fill((255, 255, 255))  # Clear screen with white
        # Here you can add code to visualize LiDAR data and robot position
        pygame.display.flip()

    def cleanup(self):
        pygame.quit()

if __name__ == "__main__":
    manual_control = ManualControl()
    try:
        manual_control.run()
    finally:
        manual_control.cleanup()
# File: /raspberry-pi-robot/raspberry-pi-robot/src/main.py

import time
import pygame
from sensors.imu import IMU
from sensors.lidar import Lidar
from actuators.motors import Motors
from control.manual_control import ManualControl
from control.autonomous_control import AutonomousControl
from visualization.display import Display

def main():
    # Initialize Pygame for visualization
    pygame.init()
    
    # Set up the display
    display = Display()
    
    # Initialize sensors
    imu = IMU()
    lidar = Lidar()
    
    # Initialize motors
    motors = Motors()
    
    # Set up control systems
    manual_control = ManualControl(motors, lidar)
    autonomous_control = AutonomousControl(motors, imu, lidar)
    
    # Main loop
    running = True
    while running:
        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        # Update sensor data
        imu.update()
        lidar.update()
        
        # Check for manual control input
        if manual_control.is_active():
            manual_control.handle_input()
        else:
            autonomous_control.navigate()
        
        # Visualize the data
        display.update(lidar.get_data(), imu.get_orientation(), motors.get_status())
        
        # Delay to maintain a consistent loop rate
        time.sleep(0.1)
    
    # Clean up
    pygame.quit()

if __name__ == "__main__":
    main()
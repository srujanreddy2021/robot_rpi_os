# Raspberry Pi Autonomous Robot Project

## Overview
This project involves building an autonomous robot using a Raspberry Pi 5, equipped with various sensors and actuators. The robot is designed to navigate its environment using data from an IMU (MPU6050) and a LiDAR (RPLidar A1), while also allowing for manual control. The project aims to implement SLAM (Simultaneous Localization and Mapping) for accurate positioning and obstacle avoidance.

## Hardware Components
- **Raspberry Pi 5**: The main processing unit running Raspberry Pi OS.
- **RPLidar A1**: A 360-degree laser scanner for mapping and obstacle detection.
- **MPU6050**: An IMU for measuring orientation and motion.
- **Servo Motors**: For movement control of the robot.
- **Camera**: For capturing images or video for navigation and obstacle detection.

## Project Structure
The project is organized into several directories and files, each serving a specific purpose:

- **src/**: Contains the main application code.
  - **main.py**: Entry point of the application.
  - **config.py**: Configuration settings for the robot.
  - **sensors/**: Handles sensor initialization and data retrieval.
  - **actuators/**: Controls the robot's movement.
  - **navigation/**: Implements SLAM and path planning.
  - **control/**: Manages manual and autonomous control.
  - **visualization/**: Visualizes sensor data and robot movement.
  - **utils/**: Contains utility functions and filters.

- **tests/**: Contains unit tests for various components of the robot.

- **scripts/**: Includes scripts for calibration and testing.

- **requirements.txt**: Lists the required Python libraries for the project.

## Setup Instructions
1. **Install Raspberry Pi OS**: Ensure that Raspberry Pi OS is installed on your Raspberry Pi 5.
2. **Clone the Repository**: Clone this project repository to your local machine.
3. **Install Dependencies**: Navigate to the project directory and run:
   ```
   pip install -r requirements.txt
   ```
4. **Connect Hardware**: Connect the RPLidar, MPU6050, motors, and camera to the Raspberry Pi according to the wiring diagrams provided in the respective sensor documentation.
5. **Run the Robot**: Start the robot by executing:
   ```
   python src/main.py
   ```

## Usage Guidelines
- **Manual Control**: Use keyboard inputs to control the robot's movement manually.
- **Autonomous Navigation**: The robot can autonomously navigate to specified coordinates while avoiding obstacles.
- **Visualization**: The robot's LiDAR output, detected obstacles, and SLAM-based map can be visualized in real-time using Pygame.

## Capabilities
- **Obstacle Detection**: The robot can detect obstacles in its path using LiDAR data.
- **SLAM Implementation**: The robot can create a map of its environment while keeping track of its position.
- **Flexible Control**: Users can switch between manual and autonomous control modes.

## Future Enhancements
- Implement advanced path planning algorithms such as A*, RRT, or D* Lite for improved navigation.
- Integrate additional sensors for enhanced environmental awareness.
- Develop a web interface for remote control and monitoring of the robot.

## Acknowledgments
This project utilizes various open-source libraries and resources. Special thanks to the contributors of the libraries used for sensor interfacing and SLAM algorithms.
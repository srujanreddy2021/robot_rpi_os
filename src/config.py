# Configuration settings for the robot

# Motor speed settings
MOTOR_SPEED_FORWARD = 100  # Speed for moving forward (0-255)
MOTOR_SPEED_BACKWARD = 100  # Speed for moving backward (0-255)
MOTOR_TURN_SPEED = 75  # Speed for turning (0-255)

# Sensor thresholds
LIDAR_OBSTACLE_THRESHOLD = 30  # Distance in cm to consider an obstacle detected
IMU_CALIBRATION_SAMPLES = 1000  # Number of samples for IMU calibration

# SLAM settings
SLAM_UPDATE_RATE = 0.1  # Update rate for SLAM in seconds
SLAM_MAP_RESOLUTION = 0.05  # Resolution of the SLAM map in meters

# Camera settings
CAMERA_RESOLUTION = (640, 480)  # Resolution for the camera (width, height)
CAMERA_FPS = 30  # Frames per second for camera capture

# General settings
MAX_DISTANCE_TRAVELLED = 1000  # Maximum distance the robot can travel in cm
MIN_DISTANCE_TRAVELLED = 10  # Minimum distance to consider movement valid

# Add any additional configuration settings as needed
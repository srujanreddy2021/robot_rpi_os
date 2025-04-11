# imu.py

import time
import numpy as np
from mpu6050 import mpu6050  # pip install mpu6050-raspberrypi

class IMU:
    def __init__(self):
        # Initialize the MPU6050 sensor
        try:
            self.sensor = mpu6050(0x68)
            print("MPU6050 initialized successfully")
            
            # Perform quick calibration at startup
            self.calibrate()
        except Exception as e:
            print(f"Error initializing MPU6050: {e}")
    
    def calibrate(self):
        """Calibrate the sensor"""
        print("Calibrating IMU... Keep the sensor still.")
        
        # Collect samples for calibration
        gyro_bias_sum = [0, 0, 0]
        samples = 1000
        
        for _ in range(samples):
            gyro_data = self.sensor.get_gyro_data()
            gyro_bias_sum[0] += gyro_data['x']
            gyro_bias_sum[1] += gyro_data['y'] 
            gyro_bias_sum[2] += gyro_data['z']
            time.sleep(0.01)
        
        # Calculate average bias
        self.gyro_bias = [bias/samples for bias in gyro_bias_sum]
        print(f"Calibration complete. Gyro bias: {self.gyro_bias}")
    
    def get_orientation(self):
        """Get current orientation in degrees (roll, pitch, yaw)"""
        # Get accelerometer data
        accel = self.sensor.get_accel_data()
        
        # Calculate roll and pitch from accelerometer data
        ax, ay, az = accel['x'], accel['y'], accel['z']
        roll = round(self._get_roll(ax, ay, az), 2)
        pitch = round(self._get_pitch(ax, ay, az), 2)
        
        
        # Return the orientation (yaw is not reliable without magnetometer)
        return (roll, pitch, 0.0)
    
    def _get_roll(self, ax, ay, az):
        """Calculate roll angle from accelerometer data"""
        return 180 * np.arctan2(ay, np.sqrt(ax*ax + az*az)) / np.pi
    
    def _get_pitch(self, ax, ay, az):
        """Calculate pitch angle from accelerometer data"""
        return 180 * np.arctan2(-ax, az) / np.pi
    
    
    
    def get_motion_data(self):
        """Get all motion data in a structured format"""
        accel = self.sensor.get_accel_data()
        gyro = self.sensor.get_gyro_data()
        temp = self.sensor.get_temp()
        orientation = self.get_orientation()
        
        return {
            'acceleration': [accel['x'], accel['y'], accel['z']],
            'gyroscope': [gyro['x'] - self.gyro_bias[0], 
                         gyro['y'] - self.gyro_bias[1], 
                         gyro['z'] - self.gyro_bias[2]],
            'orientation': orientation,
            'temperature': temp
        }

if __name__ == "__main__":
    imu = IMU()
    
    try:
        while True:
            roll, pitch, yaw = imu.get_orientation()
            data = imu.get_motion_data()
            print(f"Acceleration: {data['acceleration']}")
            print(f"Gyroscope: {data['gyroscope']}")
            print(f"Temperature: {data['temperature']}°C")
            print(f"Orientation: Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°")
            # print(f"Raw Data: {data}")
            # print(f"Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nExiting...")
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
            
            # Initialize time for integration
            self.last_time = time.time()
            
            # Initialize angles
            self.roll = 0.0
            self.pitch = 0.0
            self.yaw = 0.0
            
            # Position tracking (in centimeters)
            self.position = [0.0, 0.0, 0.0]
            self.velocity = [0.0, 0.0, 0.0]
            self.last_accel = [0.0, 0.0, 0.0]
            
            # Perform quick calibration at startup
            self.calibrate()
        except Exception as e:
            print(f"Error initializing MPU6050: {e}")
    
    def calibrate(self):
        """Calibrate the sensor"""
        print("Calibrating IMU... Keep the sensor still.")
        
        # Collect samples for calibration
        gyro_bias_sum = [0, 0, 0]
        accel_bias_sum = [0, 0, 0]
        samples = 500  # Reduced sample size for quicker calibration
        
        for _ in range(samples):
            gyro_data = self.sensor.get_gyro_data()
            accel_data = self.sensor.get_accel_data()
            
            gyro_bias_sum[0] += gyro_data['x']
            gyro_bias_sum[1] += gyro_data['y'] 
            gyro_bias_sum[2] += gyro_data['z']
            
            accel_bias_sum[0] += accel_data['x']
            accel_bias_sum[1] += accel_data['y']
            accel_bias_sum[2] += accel_data['z']
            
            time.sleep(0.01)
        
        # Calculate average bias
        self.gyro_bias = [bias/samples for bias in gyro_bias_sum]
        
        # Calculate accelerometer bias (removing gravity from z-axis)
        self.accel_bias = [
            accel_bias_sum[0]/samples,
            accel_bias_sum[1]/samples,
            accel_bias_sum[2]/samples - 9.81  # Remove gravity
        ]
        
        print(f"Calibration complete.")
        print(f"Gyro bias: {self.gyro_bias}")
        print(f"Accel bias: {self.accel_bias}")
    
    def get_orientation(self):
        """Get current orientation in degrees (roll, pitch, yaw)"""
        # Get current time and calculate dt
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Get accelerometer and gyroscope data
        accel = self.sensor.get_accel_data()
        gyro = self.sensor.get_gyro_data()
        
        # Calculate roll and pitch from accelerometer data
        ax, ay, az = accel['x'], accel['y'], accel['z']
        
        # Calculate angles from accelerometer
        accel_roll = self._get_roll(ax, ay, az)
        accel_pitch = self._get_pitch(ax, ay, az)
        
        # Apply complementary filter for roll and pitch
        # (98% gyroscope data + 2% accelerometer data)
        gyro_x = gyro['x'] - self.gyro_bias[0]
        gyro_y = gyro['y'] - self.gyro_bias[1]
        gyro_z = gyro['z'] - self.gyro_bias[2]
        
        self.roll = 0.98 * (self.roll + gyro_x * dt) + 0.02 * accel_roll
        self.pitch = 0.98 * (self.pitch + gyro_y * dt) + 0.02 * accel_pitch
        
        # Integrate gyroscope data for yaw
        self.yaw += gyro_z * dt
        
        # Normalize yaw to 0-360 range
        self.yaw = self.yaw % 360
        if self.yaw < 0:
            self.yaw += 360
        
        # Update position
        self._update_position(accel, dt)
        
        return (round(self.roll, 2), round(self.pitch, 2), round(self.yaw, 2))
    
    def _update_position(self, accel, dt):
        """Update position estimation using accelerometer data"""
        # Get acceleration in m/s²
        ax = accel['x'] - self.accel_bias[0]
        ay = accel['y'] - self.accel_bias[1]
        az = accel['z'] - self.accel_bias[2]
        
        # Simple high-pass filter to remove drift
        alpha = 0.8
        filtered_ax = alpha * (ax - self.last_accel[0])
        filtered_ay = alpha * (ay - self.last_accel[1])
        filtered_az = alpha * (az - self.last_accel[2])
        
        self.last_accel = [ax, ay, az]
        
        # Update velocity
        self.velocity[0] += filtered_ax * dt
        self.velocity[1] += filtered_ay * dt
        self.velocity[2] += filtered_az * dt
        
        # Damping factor to reduce drift
        damping = 0.95
        self.velocity = [v * damping for v in self.velocity]
        
        # Update position (convert to cm)
        self.position[0] += self.velocity[0] * dt * 100
        self.position[1] += self.velocity[1] * dt * 100
        self.position[2] += self.velocity[2] * dt * 100
        
        return self.position
    
    def _get_roll(self, ax, ay, az):
        """Calculate roll angle from accelerometer data"""
        return 180 * np.arctan2(ay, np.sqrt(ax*ax + az*az)) / np.pi
    
    def _get_pitch(self, ax, ay, az):
        """Calculate pitch angle from accelerometer data"""
        return 180 * np.arctan2(-ax, az) / np.pi
    
    def get_acceleration(self):
        """Get calibrated acceleration data (m/s²)"""
        accel = self.sensor.get_accel_data()
        return [
            accel['x'] - self.accel_bias[0],
            accel['y'] - self.accel_bias[1],
            accel['z'] - self.accel_bias[2]
        ]
    
    def get_gyroscope(self):
        """Get calibrated gyroscope data (degrees/s)"""
        gyro = self.sensor.get_gyro_data()
        return [
            gyro['x'] - self.gyro_bias[0],
            gyro['y'] - self.gyro_bias[1],
            gyro['z'] - self.gyro_bias[2]
        ]
    
    def get_motion_data(self):
        """Get all motion data in a structured format"""
        accel = self.get_acceleration()
        gyro = self.get_gyroscope()
        orientation = self.get_orientation()
        temp = self.sensor.get_temp()
        
        return {
            'acceleration': accel,
            'gyroscope': gyro,
            'orientation': orientation,
            'position': [round(p, 2) for p in self.position],  # In cm
            'temperature': temp
        }

if __name__ == "__main__":
    imu = IMU()
    
    try:
        while True:
            roll, pitch, yaw = imu.get_orientation()
            data = imu.get_motion_data()
            
            print(f"Acceleration (m/s²): {[round(a, 2) for a in data['acceleration']]}")
            print(f"Gyroscope (deg/s): {[round(g, 2) for g in data['gyroscope']]}")
            print(f"Orientation: Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°")
            print(f"Position (cm): {data['position']}")
            print(f"Temperature: {data['temperature']:.1f}°C")
            print("-" * 50)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nExiting...")
# imu.py

import smbus
import time
import numpy as np

class MPU6050:
    def __init__(self, address=0x68):
        self.address = address
        self.bus = smbus.SMBus(1)
        self.initialize()

    def initialize(self):
        self.bus.write_byte_data(self.address, 0x6B, 0)  # Wake up the MPU6050
        self.bus.write_byte_data(self.address, 0x1B, 0)  # Set gyro range to ±250 degrees/s
        self.bus.write_byte_data(self.address, 0x1C, 0)  # Set accelerometer range to ±2g

    def read_raw_data(self):
        accel_x = self.bus.read_byte_data(self.address, 0x3B) << 8 | self.bus.read_byte_data(self.address, 0x3C)
        accel_y = self.bus.read_byte_data(self.address, 0x3D) << 8 | self.bus.read_byte_data(self.address, 0x3E)
        accel_z = self.bus.read_byte_data(self.address, 0x3F) << 8 | self.bus.read_byte_data(self.address, 0x40)
        gyro_x = self.bus.read_byte_data(self.address, 0x43) << 8 | self.bus.read_byte_data(self.address, 0x44)
        gyro_y = self.bus.read_byte_data(self.address, 0x45) << 8 | self.bus.read_byte_data(self.address, 0x46)
        gyro_z = self.bus.read_byte_data(self.address, 0x47) << 8 | self.bus.read_byte_data(self.address, 0x48)
        return [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]

    def get_orientation(self):
        raw_data = self.read_raw_data()
        ax, ay, az, gx, gy, gz = [x / 16384.0 for x in raw_data[:3]] + [x / 131.0 for x in raw_data[3:]]
        
        # Calculate pitch and roll
        pitch = np.arctan2(ay, np.sqrt(ax**2 + az**2)) * 180 / np.pi
        roll = np.arctan2(-ax, az) * 180 / np.pi
        yaw = np.arctan2(ay, ax) * 180 / np.pi
        
        return pitch, roll, yaw

    def kalman_filter(self, angle_measured, angle_rate, dt):
        # Placeholder for Kalman filter implementation
        # This should be replaced with a full Kalman filter algorithm
        return angle_measured + angle_rate * dt

# Example usage
if __name__ == "__main__":
    imu = MPU6050()
    while True:
        pitch, roll, yaw = imu.get_orientation()
        print(f"Pitch: {pitch:.2f}, Roll: {roll:.2f}, Yaw: {yaw:.2f}")
        time.sleep(0.1)
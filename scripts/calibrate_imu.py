import smbus2
import time
import numpy as np
import json
import os

class MPU6050:
    def __init__(self, bus_number=1, address=0x68):
        self.bus = smbus2.SMBus(bus_number)
        self.address = address
        self.accel_data = np.zeros(3)
        self.gyro_data = np.zeros(3)
        self.angle = np.zeros(3)
        self.kalman_angle = np.zeros(3)
        self.kalman_bias = np.zeros(3)
        self.kalman_P = np.eye(3) * 0.1
        self.kalman_Q = np.eye(3) * 0.01
        self.kalman_R = np.eye(3) * 0.1

        self.initialize_sensor()

    def initialize_sensor(self):
        self.bus.write_byte_data(self.address, 0x6B, 0)  # Wake up the MPU6050

    def read_raw_data(self):
        accel_x = self.bus.read_byte_data(self.address, 0x3B) << 8 | self.bus.read_byte_data(self.address, 0x3C)
        accel_y = self.bus.read_byte_data(self.address, 0x3D) << 8 | self.bus.read_byte_data(self.address, 0x3E)
        accel_z = self.bus.read_byte_data(self.address, 0x3F) << 8 | self.bus.read_byte_data(self.address, 0x40)
        gyro_x = self.bus.read_byte_data(self.address, 0x43) << 8 | self.bus.read_byte_data(self.address, 0x44)
        gyro_y = self.bus.read_byte_data(self.address, 0x45) << 8 | self.bus.read_byte_data(self.address, 0x46)
        gyro_z = self.bus.read_byte_data(self.address, 0x47) << 8 | self.bus.read_byte_data(self.address, 0x48)

        self.accel_data = np.array([accel_x, accel_y, accel_z]) / 16384.0  # Scale to g
        self.gyro_data = np.array([gyro_x, gyro_y, gyro_z]) / 131.0  # Scale to degrees/s

    def update_kalman_filter(self, dt):
        for i in range(3):
            # Predict
            self.kalman_angle[i] += dt * (self.gyro_data[i] - self.kalman_bias[i])
            self.kalman_P[i][i] += self.kalman_Q[i][i]

            # Update
            y = self.accel_data[i]  # Measurement
            S = self.kalman_P[i][i] + self.kalman_R[i][i]
            K = self.kalman_P[i][i] / S
            self.kalman_angle[i] += K * (y - self.kalman_angle[i])
            self.kalman_bias[i] += K * (y - self.kalman_angle[i])
            self.kalman_P[i][i] -= K * self.kalman_P[i][i]

    def get_orientation(self):
        return self.kalman_angle

def calibrate_imu():
    imu = MPU6050()
    print("Calibrating IMU...")

    # Collect data for calibration
    num_samples = 1000
    gyro_bias = np.zeros(3)
    accel_bias = np.zeros(3)

    for _ in range(num_samples):
        imu.read_raw_data()
        gyro_bias += imu.gyro_data
        accel_bias += imu.accel_data
        time.sleep(0.01)

    gyro_bias /= num_samples
    accel_bias /= num_samples
    
    # Acceleration bias should have gravity removed from the z-axis
    gravity_compensated = accel_bias.copy()
    gravity_compensated[2] -= 1.0  # Remove gravity (1g) from z-axis
    
    # Save calibration data to a file
    calibration_data = {
        "gyro_bias": gyro_bias.tolist(),
        "accel_bias": gravity_compensated.tolist(),
    }
    
    # Define the calibration file path
    calibration_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "src", "sensors", "calibration")
    os.makedirs(calibration_dir, exist_ok=True)
    calibration_file = os.path.join(calibration_dir, "imu_calibration.json")
    
    with open(calibration_file, 'w') as f:
        json.dump(calibration_data, f)
    
    print(f"Calibration complete. Data saved to {calibration_file}")
    print(f"Gyro bias: {gyro_bias}")
    print(f"Accel bias: {gravity_compensated}")

if __name__ == "__main__":
    calibrate_imu()
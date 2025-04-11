import unittest
from src.sensors.imu import MPU6050

class TestIMU(unittest.TestCase):
    def setUp(self):
        self.imu = MPU6050()
        self.imu.calibrate()

    def test_get_orientation(self):
        orientation = self.imu.get_orientation()
        self.assertIsInstance(orientation, tuple)
        self.assertEqual(len(orientation), 3)  # Expecting (roll, pitch, yaw)

    def test_get_motion_data(self):
        motion_data = self.imu.get_motion_data()
        self.assertIsInstance(motion_data, dict)
        self.assertIn('acceleration', motion_data)
        self.assertIn('gyroscope', motion_data)

    def test_kalman_filter(self):
        raw_data = self.imu.get_raw_data()
        filtered_data = self.imu.apply_kalman_filter(raw_data)
        self.assertIsInstance(filtered_data, tuple)
        self.assertEqual(len(filtered_data), 3)  # Expecting (filtered_roll, filtered_pitch, filtered_yaw)

if __name__ == '__main__':
    unittest.main()
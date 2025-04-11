# Test for LiDAR Functionality

import unittest
from src.sensors.lidar import RPLidar

class TestLiDAR(unittest.TestCase):
    def setUp(self):
        # Initialize the LiDAR sensor
        self.lidar = RPLidar('/dev/ttyUSB0')  # Adjust the port as necessary

    def test_start(self):
        # Test if the LiDAR starts correctly
        self.lidar.start()
        self.assertTrue(self.lidar.is_running)

    def test_get_scan(self):
        # Test if we can get a scan from the LiDAR
        self.lidar.start()
        scan_data = self.lidar.get_scan()
        self.assertIsInstance(scan_data, list)
        self.assertGreater(len(scan_data), 0)

    def test_stop(self):
        # Test if the LiDAR stops correctly
        self.lidar.start()
        self.lidar.stop()
        self.assertFalse(self.lidar.is_running)

    def tearDown(self):
        # Clean up LiDAR resources
        self.lidar.stop()
        self.lidar.disconnect()

if __name__ == '__main__':
    unittest.main()
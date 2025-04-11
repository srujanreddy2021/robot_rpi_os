# Test Motors Module

import unittest
from src.actuators.motors import Motors

class TestMotors(unittest.TestCase):
    def setUp(self):
        self.motors = Motors()

    def test_move_forward(self):
        initial_position = self.motors.get_position()
        self.motors.move_forward(10)  # Move forward 10 cm
        new_position = self.motors.get_position()
        self.assertGreater(new_position[0], initial_position[0], "Motor did not move forward correctly.")

    def test_move_backward(self):
        initial_position = self.motors.get_position()
        self.motors.move_backward(10)  # Move backward 10 cm
        new_position = self.motors.get_position()
        self.assertLess(new_position[0], initial_position[0], "Motor did not move backward correctly.")

    def test_turn_right(self):
        initial_orientation = self.motors.get_orientation()
        self.motors.turn_right(90)  # Turn right 90 degrees
        new_orientation = self.motors.get_orientation()
        self.assertEqual((new_orientation - initial_orientation) % 360, 90, "Motor did not turn right correctly.")

    def test_turn_left(self):
        initial_orientation = self.motors.get_orientation()
        self.motors.turn_left(90)  # Turn left 90 degrees
        new_orientation = self.motors.get_orientation()
        self.assertEqual((new_orientation - initial_orientation) % 360, 270, "Motor did not turn left correctly.")

    def test_set_speed(self):
        self.motors.set_speed(50)  # Set speed to 50%
        self.assertEqual(self.motors.get_speed(), 50, "Motor speed was not set correctly.")

if __name__ == '__main__':
    unittest.main()
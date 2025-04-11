# Contents of /raspberry-pi-robot/raspberry-pi-robot/scripts/test_motors.py

import time
import sys
from src.actuators.motors import Motors

def test_motors():
    # Initialize the Motors class
    motors = Motors()

    try:
        # Test moving forward
        print("Moving forward for 2 seconds...")
        motors.move_forward(20)  # Move forward for 20 cm
        time.sleep(2)
        motors.stop()

        # Test moving backward
        print("Moving backward for 2 seconds...")
        motors.move_backward(20)  # Move backward for 20 cm
        time.sleep(2)
        motors.stop()

        # Test turning right
        print("Turning right for 1 second...")
        motors.turn_right(90)  # Turn right 90 degrees
        time.sleep(1)
        motors.stop()

        # Test turning left
        print("Turning left for 1 second...")
        motors.turn_left(90)  # Turn left 90 degrees
        time.sleep(1)
        motors.stop()

        print("Motor tests completed successfully.")

    except KeyboardInterrupt:
        print("Test interrupted by user.")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        motors.stop()  # Ensure motors are stopped on exit

if __name__ == "__main__":
    test_motors()
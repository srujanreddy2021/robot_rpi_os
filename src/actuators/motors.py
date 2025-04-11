class MotorController:
    def __init__(self, left_motor_pin, right_motor_pin):
        self.left_motor_pin = left_motor_pin
        self.right_motor_pin = right_motor_pin
        self.speed = 0  # Default speed
        self.setup_motors()

    def setup_motors(self):
        # Initialize GPIO pins for motor control
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.left_motor_pin, GPIO.OUT)
        GPIO.setup(self.right_motor_pin, GPIO.OUT)
        self.left_motor = GPIO.PWM(self.left_motor_pin, 100)  # 100 Hz
        self.right_motor = GPIO.PWM(self.right_motor_pin, 100)  # 100 Hz
        self.left_motor.start(0)  # Start with 0% duty cycle
        self.right_motor.start(0)  # Start with 0% duty cycle

    def set_speed(self, speed):
        # Set the speed of the motors (0-100)
        self.speed = max(0, min(100, speed))  # Clamp speed between 0 and 100
        self.left_motor.ChangeDutyCycle(self.speed)
        self.right_motor.ChangeDutyCycle(self.speed)

    def move_forward(self, distance_cm):
        # Move forward for a specified distance
        self.set_speed(100)  # Set speed to 100%
        # Logic to move forward for distance_cm
        # This is a placeholder for actual movement logic
        print(f"Moving forward for {distance_cm} cm")

    def move_backward(self, distance_cm):
        # Move backward for a specified distance
        self.set_speed(100)  # Set speed to 100%
        # Logic to move backward for distance_cm
        # This is a placeholder for actual movement logic
        print(f"Moving backward for {distance_cm} cm")

    def turn_right(self, degrees):
        # Turn right by a specified angle
        self.set_speed(100)  # Set speed to 100%
        # Logic to turn right for degrees
        # This is a placeholder for actual turning logic
        print(f"Turning right for {degrees} degrees")

    def turn_left(self, degrees):
        # Turn left by a specified angle
        self.set_speed(100)  # Set speed to 100%
        # Logic to turn left for degrees
        # This is a placeholder for actual turning logic
        print(f"Turning left for {degrees} degrees")

    def stop(self):
        # Stop the motors
        self.set_speed(0)
        print("Motors stopped")

    def cleanup(self):
        # Cleanup GPIO settings
        import RPi.GPIO as GPIO
        self.stop()
        GPIO.cleanup()
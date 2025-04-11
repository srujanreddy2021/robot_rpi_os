import time
import RPi.GPIO as GPIO


class MotorController:
    def __init__(self, left_motor_pin, right_motor_pin):
        self.left_motor_pin = left_motor_pin
        self.right_motor_pin = right_motor_pin
        # Initialize GPIO

        self.GPIO = GPIO
        # self.GPIO.setmode(self.GPIO.BCM)
        self.setup_motors()
        
    def setup_motors(self):
        # Initialize GPIO pins for servo control
        self.GPIO.setmode(self.GPIO.BCM)
        self.GPIO.setup(self.left_motor_pin, self.GPIO.OUT)
        self.GPIO.setup(self.right_motor_pin, self.GPIO.OUT)
        
        # For servos, typically 50Hz frequency is used
        self.left_servo = self.GPIO.PWM(self.left_motor_pin, 50)
        self.right_servo = self.GPIO.PWM(self.right_motor_pin, 50)
        
        # Start with stop position (typically around 7.5% duty cycle for servos)
        self.left_servo.start(7.5)
        self.right_servo.start(7.5)
        
        # Store the current speed values (-100 to 100, where 0 is stopped)
        self.left_speed = 0
        self.right_speed = 0

    def _convert_speed_to_duty_cycle(self, speed):
        # Convert speed (-100 to 100) to duty cycle (5 to 10)
        # 7.5 is typically the stop position
        # Values below 7.5 rotate one direction, above 7.5 rotate the other direction
        return 7.5 + (speed / 100.0 * 2.5)

    def set_speed(self, speed):
        # Set the same speed for both motors (-100 to 100)
        speed = max(-100, min(100, speed))  # Clamp speed between -100 and 100
        self.set_individual_speeds(speed, speed)
    
    def set_individual_speeds(self, left_speed, right_speed):
        # Set individual speeds for each motor
        self.left_speed = max(-100, min(100, left_speed))
        self.right_speed = max(-100, min(100, right_speed))
        
        # Convert to duty cycles and apply
        left_duty = self._convert_speed_to_duty_cycle(self.left_speed)
        # Right motor might need to be reversed depending on orientation
        right_duty = self._convert_speed_to_duty_cycle(-self.right_speed)  
        
        self.left_servo.ChangeDutyCycle(left_duty)
        self.right_servo.ChangeDutyCycle(right_duty)

    def move_forward(self, distance_cm=None):
        # Move forward - both servos rotating in the forward direction
        self.set_individual_speeds(75, 75)
        if distance_cm:
            print(f"Moving forward for {distance_cm} cm")
            # For precise distance, you would need to time the movement
            # based on speed and wheel circumference
            # import time
            # time.sleep(calculated_time)
            # self.stop()

    def move_backward(self, distance_cm=None):
        # Move backward - both servos rotating in the backward direction
        self.set_individual_speeds(-75, -75)
        if distance_cm:
            print(f"Moving backward for {distance_cm} cm")
            # Similar timing logic as move_forward would go here

    def turn_right(self, degrees=None):
        # For a right turn, left wheel forward, right wheel backward
        self.set_individual_speeds(75, -75)
        if degrees:
            print(f"Turning right for {degrees} degrees")
            # Timing would depend on robot's turning circumference
            # self.stop()

    def turn_left(self, degrees=None):
        # For a left turn, left wheel backward, right wheel forward
        self.set_individual_speeds(-75, 75)
        if degrees:
            print(f"Turning left for {degrees} degrees")
            # Timing logic would go here

    def stop(self):
        # Stop the servos by setting to neutral position
        self.set_individual_speeds(0, 0)
        print("Motors stopped")

    def cleanup(self):
        # Stop the servos first
        self.stop()
        
        # Stop the PWM outputs
        try:
            self.left_servo.stop()
            self.right_servo.stop()
        except:
            pass  # In case they were already stopped
        
        # Only clean up the pins we used instead of all GPIO pins
        try:
            self.GPIO.cleanup([self.left_motor_pin, self.right_motor_pin])
        except:
            pass  # In case GPIO was already cleaned up
        
if __name__ == "__main__":

    motor_controller = MotorController(left_motor_pin=17, right_motor_pin=27)
    try:
        while True:
            motor_controller.move_forward(100)
            time.sleep(3)
            # motor_controller.move_backward(100)
            # time.sleep(2)
        # motor_controller.turn_right(90)
        
        # motor_controller.move_backward(10)
        # time.sleep(2)
    finally:
        motor_controller.cleanup()
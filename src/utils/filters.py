# File: /raspberry-pi-robot/raspberry-pi-robot/src/utils/filters.py

"""
This file contains implementations of various filters for sensor data processing.
The primary filters included are the Kalman filter and the complementary filter.
"""

import numpy as np

class KalmanFilter:
    def __init__(self, process_variance, measurement_variance, estimated_measurement_variance):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimated_measurement_variance = estimated_measurement_variance
        self.posteri_estimate = 0.0
        self.posteri_error_estimate = 1.0

    def update(self, measurement):
        # Prediction update
        priori_estimate = self.posteri_estimate
        priori_error_estimate = self.posteri_error_estimate + self.process_variance

        # Measurement update
        blending_factor = priori_error_estimate / (priori_error_estimate + self.measurement_variance)
        self.posteri_estimate = priori_estimate + blending_factor * (measurement - priori_estimate)
        self.posteri_error_estimate = (1 - blending_factor) * priori_error_estimate

        return self.posteri_estimate


class ComplementaryFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.angle = 0.0

    def update(self, new_angle, new_rate, dt):
        # Complementary filter to combine accelerometer and gyroscope data
        self.angle = self.alpha * (self.angle + new_rate * dt) + (1 - self.alpha) * new_angle
        return self.angle


# Example usage (commented out for clarity):
# kalman_filter = KalmanFilter(process_variance=1e-5, measurement_variance=0.1, estimated_measurement_variance=1.0)
# filtered_value = kalman_filter.update(measurement)

# complementary_filter = ComplementaryFilter(alpha=0.98)
# filtered_angle = complementary_filter.update(new_angle, new_rate, dt)
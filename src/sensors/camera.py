# File: /raspberry-pi-robot/raspberry-pi-robot/src/sensors/camera.py

import cv2

class Camera:
    def __init__(self, camera_index=0):
        """Initialize the camera."""
        self.camera_index = camera_index
        self.cap = cv2.VideoCapture(self.camera_index)

        if not self.cap.isOpened():
            raise Exception("Could not open video device")

    def capture_frame(self):
        """Capture a single frame from the camera."""
        ret, frame = self.cap.read()
        if not ret:
            raise Exception("Could not read frame from camera")
        return frame

    def release(self):
        """Release the camera resource."""
        self.cap.release()

    def show_frame(self, frame):
        """Display the captured frame."""
        cv2.imshow('Camera Frame', frame)
        cv2.waitKey(1)  # Display the frame for 1 ms

    def __del__(self):
        """Ensure the camera is released when the object is deleted."""
        self.release()
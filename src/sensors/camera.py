# File: /raspberry-pi-robot/raspberry-pi-robot/src/sensors/camera.py

# Camera module for the robot
import cv2
import time
import threading
import numpy as np

class Camera:
    def __init__(self, camera_id=0, width=640, height=480, fps=30):
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.fps = fps
        
        # Initialize camera
        self.cap = cv2.VideoCapture(camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        
        # Frame storage
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        
        # Start capture thread
        self.running = True
        self.capture_thread = threading.Thread(target=self._capture_loop)
        self.capture_thread.daemon = True
        self.capture_thread.start()
    
    def _capture_loop(self):
        """Background thread to continuously capture frames"""
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.frame_lock:
                    self.latest_frame = frame
            time.sleep(1/self.fps)  # Control capture rate
    
    def get_frame(self):
        """Get the latest captured frame"""
        with self.frame_lock:
            if self.latest_frame is not None:
                return self.latest_frame.copy()
            return None
    
    def stop(self):
        """Stop the camera and release resources"""
        self.running = False
        time.sleep(0.5)  # Allow thread to clean up
        self.cap.release()
        
    def __del__(self):
        self.stop()

if __name__ == "__main__":
    # Test the camera module
    camera = Camera()
    try:
        while True:
            frame = camera.get_frame()
            if frame is not None:
                cv2.imshow('Camera Test', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            time.sleep(0.03)
    except KeyboardInterrupt:
        print("Camera test stopped by user")
    finally:
        camera.stop()
        cv2.destroyAllWindows()
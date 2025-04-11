# slam.py
import sys
import os
import threading
import time
import cv2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Arrow
from collections import deque
from scipy.spatial import KDTree
from scipy.ndimage import gaussian_filter
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sensors.lidar import Lidar
from sensors.imu import IMU
from sensors.camera import Camera  # You'll need to create this camera module

class SLAM:
    def __init__(self):
        # Initialize sensors
        self.lidar = Lidar()
        self.imu = IMU()
        self.camera = Camera()  # Initialize camera
        
        # Initialize position tracking variables (in centimeters)
        self.position = np.array([0.0, 0.0])  # Robot's position (x, y) in cm
        self.velocity = np.array([0.0, 0.0])  # Robot's velocity (vx, vy) in cm/s
        self.orientation = 0.0  # Robot's orientation in degrees
        self.map = []  # List to hold LiDAR map points
        self.visual_features = []  # List to hold camera visual features
        
        # Grid map parameters
        self.grid_size = 5  # Grid cell size in cm
        self.grid_width = 1000  # Grid width in cells
        self.grid_height = 1000  # Grid height in cells
        self.grid_origin = np.array([self.grid_width//2, self.grid_height//2])  # Center of the grid
        self.occupancy_grid = np.zeros((self.grid_height, self.grid_width), dtype=np.float32)  # Occupancy grid
        
        # Keep track of path history
        self.path_history = deque(maxlen=500)  # Store last 500 positions
        
        # Keep track of IMU history
        self.imu_history = deque(maxlen=100)  # Store last 100 IMU readings
        
        # Previous scan for scan matching
        self.prev_scan = []
        
        # State uncertainty (for Kalman filter)
        self.position_uncertainty = np.array([10.0, 10.0])  # Initial position uncertainty (cm)
        self.orientation_uncertainty = 5.0  # Initial orientation uncertainty (degrees)
        
        # Thread control variables
        self.running = False
        self.lock = threading.Lock()  # Lock for thread-safe access to shared data
        
        # Visualization settings
        self.fig = None
        self.axes = None  # Will hold all subplot axes
        
    def setup_visualization(self):
        """Setup visualization in the main thread with 4 panels"""
        plt.ion()  # Interactive mode on
        self.fig = plt.figure(figsize=(16, 12))
        
        # Create a 2x2 grid of subplots
        self.axes = {
            'map': self.fig.add_subplot(221),  # Top-left: Map view with path
            'lidar': self.fig.add_subplot(222),  # Top-right: LiDAR points
            'camera': self.fig.add_subplot(223),  # Bottom-left: Camera features
            'imu': self.fig.add_subplot(224)  # Bottom-right: IMU data
        }
        
        # Set titles for each subplot
        self.axes['map'].set_title('Robot Path and Occupancy Grid (cm)')
        self.axes['lidar'].set_title('Current LiDAR Scan (cm)')
        self.axes['camera'].set_title('Camera Feature Detection')
        self.axes['imu'].set_title('IMU Data')
        
        plt.tight_layout()

    def start(self):
        """Start all sensor threads and processing"""
        self.running = True
        
        # Create and start threads for each sensor
        self.lidar_thread = threading.Thread(target=self.lidar_processing)
        self.imu_thread = threading.Thread(target=self.imu_processing)
        self.camera_thread = threading.Thread(target=self.camera_processing)
        
        self.lidar_thread.daemon = True
        self.imu_thread.daemon = True
        self.camera_thread.daemon = True
        
        # Start the threads
        self.lidar_thread.start()
        self.imu_thread.start()
        self.camera_thread.start()
    
    def stop(self):
        """Stop all threads and cleanup"""
        self.running = False
        time.sleep(1)  # Give threads time to clean up
        self.lidar.stop()
        plt.close('all')
        
    def lidar_processing(self):
        """Thread function for processing LiDAR data"""
        self.lidar.start()
        while self.running:
            try:
                self.update_map()
                time.sleep(0.1)  # Control the processing rate
            except Exception as e:
                print(f"Error in LiDAR processing: {e}")
    
    def imu_processing(self):
        """Thread function for processing IMU data"""
        while self.running:
            try:
                self.update_position()
                time.sleep(0.01)  # IMU updates at higher rate
            except Exception as e:
                print(f"Error in IMU processing: {e}")
    
    def camera_processing(self):
        """Thread function for processing camera data"""
        while self.running:
            try:
                frame = self.camera.get_frame()
                if frame is not None:
                    features = self.extract_visual_features(frame)
                    with self.lock:
                        self.visual_features = features
                time.sleep(0.1)  # Control the processing rate
            except Exception as e:
                print(f"Error in camera processing: {e}")
    
    def extract_visual_features(self, frame):
        """Extract visual features from camera frame using ORB detector"""
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Initialize ORB detector
        orb = cv2.ORB_create(nfeatures=500)
        
        # Find keypoints and descriptors
        keypoints, descriptors = orb.detectAndCompute(gray, None)
        
        # Convert keypoints to list of (x, y) coordinates
        features = [(kp.pt[0], kp.pt[1], kp.response) for kp in keypoints]
        return features
    
    def update_position(self):
        """Update position based on IMU data with Kalman filtering"""
        accel_data = self.imu.get_acceleration()
        gyro_data = self.imu.get_gyroscope()
        
        # Time step
        dt = 0.01  # 10ms update rate
        
        with self.lock:
            # ---- Prediction step ----
            # Update orientation based on gyroscope with noise consideration
            orient_change = gyro_data[2] * dt
            self.orientation += orient_change
            self.orientation = self.orientation % 360  # Normalize to [0, 360)
            
            # Update orientation uncertainty
            self.orientation_uncertainty += 0.1 * dt  # Add process noise
    
            # Convert acceleration to robot frame
            accel_robot = np.array([
                accel_data[0] * np.cos(np.radians(self.orientation)) - 
                accel_data[1] * np.sin(np.radians(self.orientation)),
                accel_data[0] * np.sin(np.radians(self.orientation)) + 
                accel_data[1] * np.cos(np.radians(self.orientation))
            ]) * 100  # Convert to cm/s²
            
            # Update velocity (v = v0 + a*t)
            self.velocity += accel_robot * dt
            
            # Apply damping to velocity (simulating friction)
            self.velocity *= 0.95  # Damping factor
            
            # Update position (x = x0 + v*t + 0.5*a*t²)
            translation = self.velocity * dt + 0.5 * accel_robot * dt * dt
            self.position += translation
            
            # Update position uncertainty
            self.position_uncertainty += 1.0 * dt  # Add process noise
            
            # Add current position to path history
            self.path_history.append(self.position.copy())
            
            # Store IMU data for visualization
            self.imu_history.append({
                'accel': accel_data.copy(), 
                'gyro': gyro_data.copy(),
                'timestamp': time.time()
            })
    
    def scan_matching(self, current_scan):
        """Perform simple ICP-like scan matching to refine position"""
        if not self.prev_scan or not current_scan:
            self.prev_scan = current_scan
            return np.array([0.0, 0.0]), 0.0
        
        # Check if there are enough points for matching
        if len(current_scan) < 10 or len(self.prev_scan) < 10:
            return np.array([0.0, 0.0]), 0.0
        
        # Convert to numpy arrays for vectorized operations
        prev_points = np.array(self.prev_scan)
        current_points = np.array(current_scan)
        
        # Build KD-Tree for nearest neighbor search
        tree = KDTree(prev_points)
        
        # Find closest points
        distances, indices = tree.query(current_points, k=1)
        
        # Filter out bad matches
        good_matches = distances < 50.0  # 50 cm threshold
        if np.sum(good_matches) < 5:
            return np.array([0.0, 0.0]), 0.0
            
        src_pts = current_points[good_matches]
        dst_pts = prev_points[indices[good_matches]]
        
        # Compute centroid
        src_centroid = np.mean(src_pts, axis=0)
        dst_centroid = np.mean(dst_pts, axis=0)
        
        # Center the point sets
        src_centered = src_pts - src_centroid
        dst_centered = dst_pts - dst_centroid
        
        # Compute rotation using SVD
        H = np.dot(src_centered.T, dst_centered)
        U, _, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)
        
        # Check if we have a reflection case
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = np.dot(Vt.T, U.T)
        
        # Extract rotation angle
        angle = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
        
        # Compute translation
        translation = dst_centroid - np.dot(src_centroid, R)
        
        # Update previous scan with current scan
        self.prev_scan = current_scan
        
        return translation, angle
    
    def update_occupancy_grid(self, scan_points):
        """Update the occupancy grid based on LiDAR scan points"""
        if not scan_points:
            return
            
        # For all scan points, update the occupancy grid
        for point in scan_points:
            # Convert world coordinates to grid coordinates
            grid_x = int(self.grid_origin[0] + point[0] / self.grid_size)
            grid_y = int(self.grid_origin[1] + point[1] / self.grid_size)
            
            # Check if the grid coordinates are within the grid
            if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
                # Increase occupancy value
                self.occupancy_grid[grid_y, grid_x] = min(1.0, self.occupancy_grid[grid_y, grid_x] + 0.1)
                
        # Add some blur to enhance visualization
        self.occupancy_grid = gaussian_filter(self.occupancy_grid, sigma=1.0)
        
    def update_map(self):
        """Update map based on LiDAR data with scan matching and occupancy grid"""
        lidar_data = self.lidar.get_scan()
        new_points = []
        scan_points_local = []  # Points in robot's local frame
        
        for angle, distance in lidar_data:
            # Convert to local Cartesian coordinates
            distance_cm = distance * 100  # Convert m to cm
            # Local coordinates in robot frame
            x_local = distance_cm * np.cos(np.radians(angle))
            y_local = distance_cm * np.sin(np.radians(angle))
            scan_points_local.append((x_local, y_local))
            
            # Convert to world coordinates
            abs_angle = angle + self.orientation
            x_world = self.position[0] + distance_cm * np.cos(np.radians(abs_angle))
            y_world = self.position[1] + distance_cm * np.sin(np.radians(abs_angle))
            new_points.append((x_world, y_world))
        
        # Perform scan matching to adjust position
        translation, angle = self.scan_matching(scan_points_local)
        
        with self.lock:
            # Update position and orientation based on scan matching
            if abs(translation[0]) < 20 and abs(translation[1]) < 20 and abs(angle) < 10:
                # Only apply corrections if they are reasonable
                # Apply corrections with weights based on uncertainties
                position_weight = 0.3
                orientation_weight = 0.2
                
                # Apply correction
                self.position[0] += translation[0] * position_weight
                self.position[1] += translation[1] * position_weight
                self.orientation += angle * orientation_weight
                
                # Reduce uncertainties based on scan matching confidence
                confidence = 1.0 / (1.0 + np.sum(np.abs(translation)) + np.abs(angle))
                self.position_uncertainty *= (1.0 - confidence * 0.2)
                self.orientation_uncertainty *= (1.0 - confidence * 0.2)
            
            # Update the map with new scan points
            self.map.extend(new_points)
            
            # Update the occupancy grid
            self.update_occupancy_grid(new_points)
            
            # Limit map size to avoid memory issues
            if len(self.map) > 10000:
                self.map = self.map[-10000:]
    
    def visualize(self):
        """Visualize robot position, direction, LiDAR data and camera features"""
        with self.lock:
            # Make local copies of shared data to avoid race conditions
            position = self.position.copy()
            orientation = self.orientation
            map_data = self.map.copy() if self.map else []
            visual_features = self.visual_features.copy() if self.visual_features else []
            path_history = list(self.path_history)
            imu_history = list(self.imu_history)
            occupancy_grid = self.occupancy_grid.copy()
        
        # Clear all plots
        for ax in self.axes.values():
            ax.clear()
            
        # ---- 1. Plot Map with Path and Occupancy Grid (top-left) ----
        # Show occupancy grid as a heatmap
        grid_extent = [
            -self.grid_origin[0] * self.grid_size, 
            (self.grid_width - self.grid_origin[0]) * self.grid_size,
            -self.grid_origin[1] * self.grid_size, 
            (self.grid_height - self.grid_origin[1]) * self.grid_size
        ]
        
        # Plot the occupancy grid
        self.axes['map'].imshow(
            occupancy_grid, 
            cmap='Blues', 
            origin='lower',
            extent=grid_extent,
            alpha=0.7
        )
        
        # Plot path history
        if path_history:
            path_x = [pos[0] for pos in path_history]
            path_y = [pos[1] for pos in path_history]
            self.axes['map'].plot(path_x, path_y, 'r-', linewidth=2, alpha=0.7)
        
        # Plot current position with uncertainty ellipse
        self.axes['map'].scatter(position[0], position[1], c='red', s=100, marker='o')
        
        # Draw uncertainty ellipse
        confidence_ellipse(
            position[0], position[1], 
            self.position_uncertainty[0], self.position_uncertainty[1], 
            orientation, self.axes['map'], 
            edgecolor='red', facecolor='none', alpha=0.5
        )
        
        # Draw arrow to show robot orientation/direction
        arrow_length = 50  # 50 cm arrow length
        dx = arrow_length * np.cos(np.radians(orientation))
        dy = arrow_length * np.sin(np.radians(orientation))
        self.axes['map'].arrow(position[0], position[1], dx, dy, 
                              head_width=20, head_length=30, 
                              fc='red', ec='red')
        
        # Set map view properties
        map_width = 800  # Show area in cm
        self.axes['map'].set_xlim(position[0]-map_width/2, position[0]+map_width/2)
        self.axes['map'].set_ylim(position[1]-map_width/2, position[1]+map_width/2)
        self.axes['map'].set_title(f'Robot Path and Occupancy Grid (cm) - Pos: ({position[0]:.1f}, {position[1]:.1f})')
        self.axes['map'].set_xlabel('X Position (cm)')
        self.axes['map'].set_ylabel('Y Position (cm)')
        self.axes['map'].grid(True)
        
        # ---- 2. Plot current LiDAR scan (top-right) ----
        if map_data:
            # Get the latest scan points (last 100 points)
            latest_scan = map_data[-100:] if len(map_data) > 100 else map_data
            x_lidar, y_lidar = zip(*latest_scan)
            
            # Create a 2D histogram (density plot) of LiDAR points
            heatmap, xedges, yedges = np.histogram2d(
                x_lidar, y_lidar, 
                bins=50, 
                range=[[position[0]-400, position[0]+400], [position[1]-400, position[1]+400]]
            )
            extent = [xedges[0], xedges[-1], yedges[0], yedges[-1]]
            
            # Plot LiDAR heat map
            self.axes['lidar'].imshow(
                heatmap.T, 
                origin='lower', 
                extent=extent, 
                cmap='plasma',
                aspect='auto',
                alpha=0.7
            )
            
            # Plot LiDAR points
            self.axes['lidar'].scatter(x_lidar, y_lidar, c='blue', s=5, alpha=0.5)
            self.axes['lidar'].scatter(position[0], position[1], c='red', s=100, marker='o')
            
            # Draw arrow to show robot orientation/direction
            self.axes['lidar'].arrow(position[0], position[1], dx, dy, 
                                   head_width=20, head_length=30, 
                                   fc='red', ec='red')
            
            # Set LiDAR view properties
            self.axes['lidar'].set_xlim(position[0]-400, position[0]+400)
            self.axes['lidar'].set_ylim(position[1]-400, position[1]+400)
            self.axes['lidar'].set_title(f'LiDAR Scan (cm) - Orientation: {orientation:.1f}°')
            self.axes['lidar'].set_xlabel('X Position (cm)')
            self.axes['lidar'].set_ylabel('Y Position (cm)')
            self.axes['lidar'].grid(True)
        
        # ---- 3. Plot camera feature data (bottom-left) ----
        if visual_features:
            x_feat, y_feat, response = zip(*visual_features)
            # Normalize response values for color mapping
            response_norm = [r/max(response) if max(response) > 0 else r for r in response]
            
            # Create a frame to show camera field of view
            camera_frame = np.zeros((480, 640, 3), dtype=np.uint8)
            
            # Draw feature points on the frame
            for i, (x, y, r) in enumerate(visual_features):
                color = plt.cm.viridis(response_norm[i])[:3]
                cv2.circle(
                    camera_frame, 
                    (int(x), int(y)), 
                    int(3 + r/50), 
                    (color[0]*255, color[1]*255, color[2]*255), 
                    -1
                )
            
            # Display the camera frame with features
            self.axes['camera'].imshow(camera_frame)
            
            # Set camera view properties
            self.axes['camera'].set_title(f'Camera Feature Detection - {len(visual_features)} features')
            self.axes['camera'].set_xlabel('X (pixels)')
            self.axes['camera'].set_ylabel('Y (pixels)')
            self.axes['camera'].axis('off')  # Hide axes for better visualization
            
        # ---- 4. Plot IMU data (bottom-right) ----
        if imu_history:
            # Extract time values relative to the first entry
            if imu_history:
                base_time = imu_history[0]['timestamp']
                times = [(entry['timestamp'] - base_time) for entry in imu_history]
                
                # Extract acceleration data
                accel_x = [entry['accel'][0] for entry in imu_history]
                accel_y = [entry['accel'][1] for entry in imu_history]
                accel_z = [entry['accel'][2] for entry in imu_history]
                
                # Extract gyroscope data (angular velocity)
                gyro_z = [entry['gyro'][2] for entry in imu_history]  # Yaw rate
                
                # Plot acceleration data
                self.axes['imu'].plot(times, accel_x, 'r-', label='Accel X')
                self.axes['imu'].plot(times, accel_y, 'g-', label='Accel Y')
                self.axes['imu'].plot(times, accel_z, 'b-', label='Accel Z')
                
                # Plot gyroscope data on a secondary y-axis
                ax2 = self.axes['imu'].twinx()
                ax2.plot(times, gyro_z, 'm-', label='Yaw Rate')
                ax2.set_ylabel('Angular Velocity (deg/s)', color='m')
                
                # Plot uncertainty values
                if len(times) > 0:
                    self.axes['imu'].axhline(y=self.position_uncertainty[0]/10, color='orange', linestyle='--', alpha=0.5, label='Pos. Uncert. X')
                    self.axes['imu'].axhline(y=self.position_uncertainty[1]/10, color='cyan', linestyle='--', alpha=0.5, label='Pos. Uncert. Y')
                    ax2.axhline(y=self.orientation_uncertainty, color='yellow', linestyle='--', alpha=0.5, label='Orient. Uncert.')
                
                # Set IMU plot properties
                self.axes['imu'].set_title('IMU Data and System Uncertainty')
                self.axes['imu'].set_xlabel('Time (s)')
                self.axes['imu'].set_ylabel('Acceleration (m/s²)')
                self.axes['imu'].legend(loc='upper left')
                ax2.legend(loc='upper right')
        
        # Update layout
        plt.tight_layout()
        plt.draw()
        plt.pause(0.001)

    def run(self):
        """Main method to run the SLAM system"""
        # Set up visualization in the main thread
        self.setup_visualization()
        
        # Start sensor threads
        self.start()
        
        try:
            # Keep main thread alive and handle visualization
            while self.running:
                # Update visualization directly in the main thread
                self.visualize()
                time.sleep(0.2)  # Update visualization at 5 Hz
        except KeyboardInterrupt:
            print("SLAM system stopped by user")
        finally:
            self.stop()


def confidence_ellipse(x, y, dx, dy, angle, ax, **kwargs):
    """
    Draw a confidence ellipse representing uncertainty
    """
    ellipse = plt.matplotlib.patches.Ellipse(
        (x, y), width=dx*2, height=dy*2, angle=angle, **kwargs
    )
    return ax.add_patch(ellipse)


if __name__ == "__main__":
    slam = SLAM()
    slam.run()
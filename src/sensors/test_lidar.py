from rplidar import RPLidar
import matplotlib.pyplot as plt
import numpy as np

# Initialize the plot in polar coordinates
plt.figure(figsize=(8, 8))
ax = plt.subplot(111, projection='polar')
ax.set_title('LIDAR Scan')
line, = ax.plot([], [], 'r.', markersize=2)
ax.set_rmax(5000)  # Adjust based on your LIDAR's range
ax.grid(True)

lidar = RPLidar('/dev/ttyUSB0')

info = lidar.get_info()
print(info)

health = lidar.get_health()
print(health)

try:
    for i, scan in enumerate(lidar.iter_scans()):
        print('%d: Got %d measurments' % (i, len(scan)))
        
        # Extract angles and distances
        angles = np.array([measurement[1] * np.pi / 180 for measurement in scan])
        distances = np.array([measurement[2] for measurement in scan])
        
        # Update plot data
        line.set_xdata(angles)
        line.set_ydata(distances)
        
        # Redraw the plot
        plt.draw()
        plt.pause(0.001)  # Small pause for plot to update
        
        if i > 100:
            break
finally:
    # Clean up
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    plt.show()  # Keep the plot open
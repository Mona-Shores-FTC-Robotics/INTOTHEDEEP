import pandas as pd
import matplotlib.pyplot as plt

# Load CSV data
data = pd.read_csv("auto.csv")

# Drop columns with mostly missing values
data = data.dropna(axis=1, how='all')

# Fill remaining NaN values with zero
data = data.fillna(0)

# Convert timestamp to relative time
data['relative_time'] = (data['timestamp'] - data['timestamp'].min()) / 1e6  # Convert microseconds to seconds

# Visualize forward velocity over time
plt.figure(figsize=(10, 6))
plt.plot(data['relative_time'], data['DRIVE_COMMAND_forwardVelocity'], label='Forward Velocity')
plt.xlabel("Time (s)")
plt.ylabel("Forward Velocity")
plt.title("Forward Velocity Over Time")
plt.legend()
plt.show()

# Scatter plot for position (x, y)
plt.figure(figsize=(8, 8))
plt.scatter(data['ESTIMATED_POSE_x'], data['ESTIMATED_POSE_y'], c=data['relative_time'], cmap='viridis', s=10)
plt.colorbar(label='Time (s)')
plt.xlabel("Position X")
plt.ylabel("Position Y")
plt.title("Robot Path")
plt.show()

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load CSV data
data = pd.read_csv("auto.csv")

# Drop columns with mostly missing values
data = data.dropna(axis=1, how='all')

# Fill remaining NaN values with zero
data = data.fillna(0)

# Convert timestamp to relative time
data['relative_time'] = (data['timestamp'] - data['timestamp'].min()) / 1e6  # Convert microseconds to seconds

# Extract pose data
x = data['ESTIMATED_POSE_x']
y = data['ESTIMATED_POSE_y']
time = data['relative_time']

# Create a simplified pose map
plt.figure(figsize=(10, 8))
plt.scatter(x, y, c=time, cmap='viridis', s=20, label='Robot Pose')

# Set field boundaries and orientation
plt.xlim(-72, 72)
plt.ylim(-72, 72)
plt.axhline(0, color='black', linewidth=0.5, linestyle='--')  # Horizontal centerline
plt.axvline(0, color='black', linewidth=0.5, linestyle='--')  # Vertical centerline

# Labels and title
plt.xlabel("X Position (inches)")
plt.ylabel("Y Position (inches)")
plt.title("Robot Pose Map Over Time")
plt.colorbar(label='Time (s)')
plt.legend()
plt.grid(True)

# Add field annotations for better orientation
plt.text(-72, 0, 'Left Wall (-72 X)', va='center', ha='right', fontsize=10, color='blue')
plt.text(72, 0, 'Right Wall (72 X)', va='center', ha='left', fontsize=10, color='blue')
plt.text(0, -72, 'Bottom Wall (-72 Y)', va='top', ha='center', fontsize=10, color='blue')
plt.text(0, 72, 'Top Wall (72 Y)', va='bottom', ha='center', fontsize=10, color='blue')

plt.show()

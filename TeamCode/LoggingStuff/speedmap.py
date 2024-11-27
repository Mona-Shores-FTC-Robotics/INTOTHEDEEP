import pandas as pd
import matplotlib.pyplot as plt

# Load the data
data = pd.read_csv("robotics_log.csv")

# Drop columns with mostly missing values
data = data.dropna(axis=1, how='all')

# Convert pose data to numeric and interpolate missing values
data['ESTIMATED_POSE_x'] = pd.to_numeric(data['ESTIMATED_POSE_x'], errors='coerce').interpolate(method='linear')
data['ESTIMATED_POSE_y'] = pd.to_numeric(data['ESTIMATED_POSE_y'], errors='coerce').interpolate(method='linear')

# Convert forward velocity to numeric
data['DRIVE_COMMAND_forwardVelocity'] = pd.to_numeric(data['DRIVE_COMMAND_forwardVelocity'], errors='coerce')

# Filter for high and low velocity points
valid_pose_data = data['ESTIMATED_POSE_x'].notna() & data['ESTIMATED_POSE_y'].notna()
high_forward_velocity = data['DRIVE_COMMAND_forwardVelocity'] > 40
low_forward_velocity = data['DRIVE_COMMAND_forwardVelocity'] < -40

high_velocity_points = data[high_forward_velocity & valid_pose_data]
low_velocity_points = data[low_forward_velocity & valid_pose_data]

# Debug: Print high and low velocity points
print("\nHigh Forward Velocity Points (v > 40):")
print(high_velocity_points[['timestamp', 'DRIVE_COMMAND_forwardVelocity', 'ESTIMATED_POSE_x', 'ESTIMATED_POSE_y']])

print("\nLow Forward Velocity Points (v < -40):")
print(low_velocity_points[['timestamp', 'DRIVE_COMMAND_forwardVelocity', 'ESTIMATED_POSE_x', 'ESTIMATED_POSE_y']])

# Extract pose data
x = data['ESTIMATED_POSE_x']
y = data['ESTIMATED_POSE_y']

# Plot the pose map
plt.figure(figsize=(10, 8))

# Normal points
plt.scatter(x, y, c='gray', s=20, label='Normal Forward Velocity')

# High forward velocity points
plt.scatter(
    high_velocity_points['ESTIMATED_POSE_x'], high_velocity_points['ESTIMATED_POSE_y'], 
    c='red', s=100, label='High Forward Velocity (v > 40)'
)

# Low forward velocity points
plt.scatter(
    low_velocity_points['ESTIMATED_POSE_x'], low_velocity_points['ESTIMATED_POSE_y'], 
    c='blue', s=100, label='Low Forward Velocity (v < -40)'
)

# Set field boundaries
plt.xlim(-72, 72)
plt.ylim(-72, 72)
plt.axhline(0, color='black', linewidth=0.5, linestyle='--')  # Horizontal centerline
plt.axvline(0, color='black', linewidth=0.5, linestyle='--')  # Vertical centerline

# Labels and title
plt.xlabel("X Position (inches)")
plt.ylabel("Y Position (inches)")
plt.title("Robot Pose Map with High (v > 40) and Low (v < -40) Forward Velocity Highlights")
plt.legend()
plt.grid(True)

plt.show()

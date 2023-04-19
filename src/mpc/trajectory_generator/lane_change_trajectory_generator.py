#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
import pandas as pd
import os  

# Global Parameters
lane_width = 3.5  # meters
pre_lane_change_distance = 30  # meters
post_lane_change_distance = 30  # meters
sampling_time_interval = 0.2  # seconds
max_speed = 35  # meters per second
max_acceleration = 2  # meters per second squared
max_turning_rate = 0.5  # radians per second


# Trajectory Generation Function
def generate_trajectory(initial_speed, lane_change_time, lane_change_distance, max_speed, max_acceleration, max_turning_rate, sampling_time_interval):
    # Calculate the number of samples
    num_samples = int(lane_change_time / sampling_time_interval) + 1

    # Time vector
    t = np.linspace(0, lane_change_time, num_samples)

    # Define initial, control, and final positions
    pre_lane_change_position = np.array([pre_lane_change_distance, 0])
    initial_lane_change_position = np.array([pre_lane_change_distance, 0])
    control_position1 = np.array([pre_lane_change_distance + lane_change_distance / 4, 0.1*lane_width])
    control_position2 = np.array([pre_lane_change_distance + lane_change_distance / 2, 0.5*lane_width])
    control_position3 = np.array([pre_lane_change_distance + 3 * lane_change_distance / 4, 0.9*lane_width])
    final_lane_change_position = np.array([pre_lane_change_distance + lane_change_distance, lane_width])
    post_lane_change_position = np.array([pre_lane_change_distance + lane_change_distance + post_lane_change_distance, lane_width])

    # Cubic spline interpolation
    cs_x = CubicSpline([0, lane_change_time / 4, lane_change_time / 2, 3 * lane_change_time / 4, lane_change_time], 
                       [initial_lane_change_position[0], control_position1[0], control_position2[0], control_position3[0], final_lane_change_position[0]], 
                       bc_type=((1, 0), (1, 0)))
    cs_y = CubicSpline([0, lane_change_time / 4, lane_change_time / 2, 3 * lane_change_time / 4, lane_change_time], 
                       [initial_lane_change_position[1], control_position1[1], control_position2[1], control_position3[1], final_lane_change_position[1]], 
                       bc_type=((1, 0), (1, 0)))

    # Get the interpolated positions, velocities, and accelerations
    x = cs_x(t)
    y = cs_y(t)
    dx = cs_x(t, 1)
    dy = cs_y(t, 1)
    ddx = cs_x(t, 2)
    ddy = cs_y(t, 2)

    # Apply velocity and acceleration constraints
    for i in range(1, len(x)):
        # Calculate speed at this time step
        speed = np.sqrt(dx[i] ** 2 + dy[i] ** 2)

        # Check if speed exceeds maximum speed
        if speed > max_speed:
            # Scale velocity and acceleration to match maximum speed
            dx[i] = dx[i] * max_speed / speed
            dy[i] = dy[i] * max_speed / speed
            ddx[i] = ddx[i] * max_speed / speed
            ddy[i] = ddy[i] * max_speed / speed
  
    # Calculate the desired heading angle
    theta = np.arctan2(np.diff(y), np.diff(x))

    # Add the final heading angle to the theta array
    theta = np.append(theta, theta[-1])

    return x, y, theta


# Lane Change Urgency Time and Distance Calculation
def stopping_distance(speed, max_deceleration=max_acceleration):
    return (speed ** 2) / (2 * max_deceleration)



# Save Trajectory to CSV Function
def save_trajectory_to_csv(case, x, y, z, theta, v, change_flag, file_name):
    data = {
        'x': x,
        'y': y,
        'z': z,
        'yaw': theta,
        'velocity': v,
        'change_flag': change_flag
    }
    df = pd.DataFrame(data)
   # Set the desired directory path
    directory = os.path.expanduser('~/Aslan/testruns/Empty/LCTrajectories')

    # Create the directory if it doesn't exist
    if not os.path.exists(directory):
        os.makedirs(directory)

    # Join the directory path with the file name
    file_path = os.path.join(directory, file_name)

    # Save the DataFrame to the specified file path
    df.to_csv(file_path, index=False)
    print(f"CSV file created successfully: {file_path}")

# Test cases
test_cases = [
    {'urgency': 'Low', 'speed': 7},
    {'urgency': 'Low', 'speed': 12},
    {'urgency': 'Low', 'speed': 20},
    {'urgency': 'Medium', 'speed': 7},
    {'urgency': 'Medium', 'speed': 12},
    {'urgency': 'Medium', 'speed': 20},
    {'urgency': 'High', 'speed': 7},
    {'urgency': 'High', 'speed': 12},
    {'urgency': 'High', 'speed': 20}
]

# Calculate lane change time and distance for each urgency level
urgency_time_distance = {
    'Low': (lambda speed: (stopping_distance(speed) * 1.3, stopping_distance(speed) * 1.3 / speed)),
    'Medium': (lambda speed: (stopping_distance(speed) * 1.15, stopping_distance(speed) * 1.15 / speed)),
    'High': (lambda speed: (stopping_distance(speed), stopping_distance(speed)*2 / speed))
}

# Plot the trajectories
plt.figure(figsize=(12, 6))

# Loop through each test case
for case in test_cases:
    # Calculate lane change distance and time
    lane_change_distance, lane_change_time = urgency_time_distance[case['urgency']](case['speed'])

    # Generate the trajectory
    x, y, theta = generate_trajectory(case['speed'], lane_change_time, lane_change_distance, max_speed, max_acceleration, max_turning_rate, sampling_time_interval)

    # Calculate velocities
    # v = np.sqrt(np.diff(x)**2 + np.diff(y)**2) / sampling_time_interval
    # v = np.insert(v, 0, case['speed'])
    v = np.full_like(x, case['speed'])
    
    # Create a zeros array for the 'z' values of the lane change trajectory
    z = np.zeros_like(x)  

    # Create a zeros array for the change_flag
    change_flag = np.zeros_like(x)

    # Add straight-line segments before and after the lane change
    num_pre_points = 5
    num_post_points = 5

    pre_x = np.linspace(0, pre_lane_change_distance - sampling_time_interval, num_pre_points)
    pre_y = np.zeros_like(pre_x)
    pre_z = np.zeros_like(pre_x)
    pre_theta = np.full_like(pre_x, theta[0])
    pre_v = np.full_like(pre_x, case['speed'])
    pre_change_flag = np.zeros_like(pre_x)

    post_x = np.linspace(pre_lane_change_distance + lane_change_distance + sampling_time_interval, pre_lane_change_distance + lane_change_distance + post_lane_change_distance, num_post_points)
    post_y = np.full_like(post_x, lane_width)
    post_z = np.zeros_like(post_x)
    post_theta = np.full_like(post_x, theta[-1])
    post_v = np.full_like(post_x, case['speed'])
    post_change_flag = np.zeros_like(post_x)

    # Concatenate pre-lane change, lane change, and post-lane change segments
    x = np.concatenate((pre_x, x, post_x))
    y = np.concatenate((pre_y, y, post_y))
    z = np.concatenate((pre_z, z, post_z))
    theta = np.concatenate((pre_theta, theta, post_theta))
    v = np.concatenate((pre_v, v, post_v))
    change_flag = np.concatenate((pre_change_flag, change_flag, post_change_flag))
    
    # Plot the lane change trajectory
    plt.plot(x, y, label=f"{case['urgency']} - {case['speed']} m/s")

    # Save the trajectory to a CSV file
    file_name = f"trajectory_{case['urgency']}_{case['speed']}mps.csv"
    save_trajectory_to_csv(case, x, y, z, theta, v, change_flag, file_name)

# Format the plot
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Lane Change Trajectories')
plt.legend()
plt.grid()

# Display the plot
plt.show()


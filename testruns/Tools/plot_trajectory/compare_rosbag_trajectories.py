#!/usr/bin/env python3
import sys
import rosbag
import numpy as np
import matplotlib.pyplot as plt

# Check if the correct number of command-line arguments were provided
if len(sys.argv) != 3:
    print("Usage: python compare_trajectories.py rosbag_file1 rosbag_file2")
    sys.exit()

# Get the rosbag file paths from the command-line arguments
rosbag_file1 = sys.argv[1]
rosbag_file2 = sys.argv[2]

# Define the topic name
topic_name = "/current_pose"

# Load the data from the first rosbag file
bag1 = rosbag.Bag(rosbag_file1)
x1 = []
y1 = []
z1 = []
for topic, msg, t in bag1.read_messages(topics=[topic_name]):
    x1.append(msg.pose.position.x)
    y1.append(msg.pose.position.y)
    z1.append(msg.pose.position.z)

# Load the data from the second rosbag file
bag2 = rosbag.Bag(rosbag_file2)
x2 = []
y2 = []
z2 = []
for topic, msg, t in bag2.read_messages(topics=[topic_name]):
    x2.append(msg.pose.position.x)
    y2.append(msg.pose.position.y)
    z2.append(msg.pose.position.z)

# Plot the two trajectories
fig, ax = plt.subplots()
ax.plot(x1, y1, label="Target Trajectory")
ax.plot(x2, y2, label="Trajectory controlled by MPC")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_title("Trajectory Comparison")
ax.legend()

# Calculate the root mean square error (RMSE)
#error = np.sqrt((np.array(x1) - np.array(x2)) ** 2 +
#               (np.array(y1) - np.array(y2)) ** 2 +
#               (np.array(z1) - np.array(z2)) ** 2)
#rmse = np.sqrt(np.mean(error ** 2))
#print("RMSE: {:.3f}".format(rmse))

# Calculate the maximum error
#max_error = np.max(error)
#print("Max Error: {:.3f}".format(max_error))

# Show the plot
plt.show()

# Close the rosbag files
bag1.close()
bag2.close()


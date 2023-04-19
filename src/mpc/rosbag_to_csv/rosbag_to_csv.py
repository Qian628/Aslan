#!/usr/bin/env python2

import rosbag
import pandas as pd
import sys
import os
import tf.transformations as tft

def extract_pose_data(bag_file, topic_name):
    bag = rosbag.Bag(bag_file)
    data = []

    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        # Extract quaternion
        quaternion = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        euler = tft.euler_from_quaternion(quaternion)
        yaw = euler[2]

        data.append({
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "z": msg.pose.position.z,
            "yaw": yaw,
            "velocity": 0,
            "change_flag": 0
        })

    bag.close()
    return data

def main():
    if len(sys.argv) != 3:
        print("Usage: rosbag_to_csv.py <rosbag_file> <output_csv_file>")
        exit(1)

    bag_file = sys.argv[1]
    csv_file = sys.argv[2]

    if not os.path.exists(bag_file):
        print("Error: File '{}' not found.".format(bag_file))
        exit(1)

    topic_name = "/current_pose"
    data = extract_pose_data(bag_file, topic_name)

    if not data:
        print("Error: Topic '{}' not found in the bag file.".format(topic_name))
        exit(1)

    df = pd.DataFrame(data, columns=["x", "y", "z", "yaw", "velocity", "change_flag"])
    df.to_csv(csv_file, index=False)
    print("Successfully saved {} messages from '{}' to '{}'".format(len(data), topic_name, csv_file))

if __name__ == "__main__":
    main()

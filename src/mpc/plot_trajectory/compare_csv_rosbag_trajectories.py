#!/usr/bin/env python3
import argparse
import matplotlib.pyplot as plt
import pandas as pd
import rosbag
from geometry_msgs.msg import PoseStamped

def plot_trajectory_from_csv(file_path):
    data = pd.read_csv(file_path)
    plt.figure(1)
    plt.plot(data['x'], data['y'], label='Reference Trajectory')

def plot_trajectory_from_rosbag(file_path):
    bag = rosbag.Bag(file_path)
    x = []
    y = []
    time_current_velocity = []
    time_twist_cmd = []
    time_sd_control = []
    speed = []
    linear_vel = []
    angular_vel = []
    steer = []
    torque = []
    print("Reading messages from rosbag...")
    for topic, msg, t in bag.read_messages():
        if topic == '/current_pose' and msg._type == 'geometry_msgs/PoseStamped':
            x.append(msg.pose.position.x)
            y.append(msg.pose.position.y)
        elif topic == '/current_velocity' and msg._type == 'geometry_msgs/TwistStamped':
            speed.append(msg.twist.linear.x)
            time_current_velocity.append(msg.header.stamp.to_sec())
        elif topic == '/twist_cmd' and msg._type == 'geometry_msgs/TwistStamped':
            linear_vel.append(msg.twist.linear.x)
            angular_vel.append(msg.twist.angular.z)
            time_twist_cmd.append(msg.header.stamp.to_sec())
        elif topic == '/sd_control' and msg._type == 'aslan_msgs/SDControl':
            print("Timestamp:", msg.header.stamp.to_sec())
            steer.append(msg.steer)
            torque.append(msg.torque)
            time_sd_control.append(msg.header.stamp.to_sec())
    plt.figure(1)
    plt.plot(x, y, label='Executed Trajectory')

    plt.figure(2)
    plt.plot(time_current_velocity,speed, label='Actual Vehicle Speed')

    plt.figure(3)
    plt.plot(time_twist_cmd,linear_vel, label='Target Linear Velocity')

    plt.figure(4)
    plt.plot(time_twist_cmd, angular_vel, label='Target Angular Velocity')

    plt.figure(5)
    plt.plot(time_sd_control, steer, label='Steering Angle [%]') # Range -100 to +100, +100 is maximum left turn, -100 is maximum right turn

    plt.figure(6)
    plt.plot(time_sd_control, torque, label='Torque [%]') # Range -100 to 100, -100 is max brake, +100 is max throttle

    bag.close()

def main():
    parser = argparse.ArgumentParser(description='Plot and compare reference and executed lane change trajectories.')
    parser.add_argument('--csv', type=str, required=True, help='Path to the reference trajectory CSV file.')
    parser.add_argument('--rosbag', type=str, required=True, help='Path to the executed trajectory rosbag file.')
    args = parser.parse_args()

    plot_trajectory_from_csv(args.csv)
    plot_trajectory_from_rosbag(args.rosbag)

    plt.figure(1)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.title('Comparison of Reference and Executed Trajectories')
    plt.grid()

    plt.figure(2)
    plt.xlabel('Time [s]')
    plt.ylabel('Speed [m/s]')
    plt.legend()
    plt.title('Vehicle Speed Over Time')
    plt.grid()

    plt.figure(3)
    plt.xlabel('Time [s]')
    plt.ylabel('Target Linear Velocity [m/s]')
    plt.legend()
    plt.title('Target Linear Velocity [m/s]')
    plt.grid()

    plt.figure(4)
    plt.xlabel('Time [s]')
    plt.ylabel('Target Angular Velocity [rad/s]')
    plt.legend()
    plt.title('Target Angular Velocity [rad/s]]')
    plt.grid()

    plt.figure(5)
    plt.xlabel('Time [s]')
    plt.ylabel('Steering Angle [%]')
    plt.legend()
    plt.title('Steering Angle [%]')
    plt.grid()

    plt.figure(6)
    plt.xlabel('Time [s]')
    plt.ylabel('Torque [%]')
    plt.legend()
    plt.title('Torque [%]')
    plt.grid()

    plt.show()

if __name__ == '__main__':
    main()
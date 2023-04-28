#!/usr/bin/env python3
import argparse
import matplotlib.pyplot as plt
import pandas as pd
import rosbag
from geometry_msgs.msg import PoseStamped
import math

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
    steering_angle = []
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
            time_twist_cmd.append(msg.header.stamp.to_sec())  # Add this line
        elif topic == '/ctrl_cmd' and msg._type == 'aslan_msgs/ControlCommandStamped':
            steering_angle_deg = msg.cmd.steering_angle * (180.0 / math.pi)
            steering_angle.append(steering_angle_deg)
        elif topic == '/sd_control' and msg._type == 'aslan_msgs/SDControl':
            steer.append(msg.steer)
            torque.append(msg.torque)
            time_sd_control.append(msg.header.stamp.to_sec())
    plt.figure(1)
    plt.plot(x, y, label='Executed Trajectory')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.title('Comparison of Reference and Executed Trajectories')
    plt.grid()

    plt.figure(2)
    plt.plot(time_current_velocity,speed, label='Actual Vehicle Speed')


    plt.figure(2)
    plt.plot(time_twist_cmd,linear_vel, label='Target Linear Velocity')
    plt.xlabel('Time [s]')
    plt.ylabel('Speed [m/s]')
    plt.legend()
    plt.title('Vehicle Speed Over Time')
    plt.grid()

    plt.figure(3)
    plt.plot(time_twist_cmd, angular_vel, label='Target Angular Velocity')
    plt.xlabel('Time [s]')
    plt.ylabel('Target Angular Velocity [rad/s]')
    plt.legend()
    plt.title('Target Angular Velocity [rad/s]')
    plt.grid()

    plt.figure(4)
    plt.plot(time_sd_control, steer, label='Steering Angle [%]') # Range -100 to +100, +100 is maximum left turn, -100 is maximum right turn
    plt.xlabel('Time [s]')
    plt.ylabel('Steering Angle [%]')
    plt.legend()
    plt.title('Steering Angle [%]')
    plt.grid()

    plt.figure(5)
    plt.plot(time_sd_control, torque, label='Torque [%]') # Range -100 to 100, -100 is max brake, +100 is max throttle
    plt.xlabel('Time [s]')
    plt.ylabel('Torque [%]')
    plt.legend()
    plt.title('Torque [%]')
    plt.grid()

    plt.figure(6)
    plt.plot(steering_angle, label='Steering Angle [deg]') 
    plt.xlabel('Time [s]')
    plt.ylabel('Steering Angle [deg]')
    plt.legend()
    plt.title('Steering Angle [deg]')
    plt.grid()

    bag.close()

def save_plots(output_folder):
    plt.figure(1)
    plt.savefig("{}/trajectory_comparison.png".format(output_folder))

    plt.figure(2)
    plt.savefig("{}/vehicle_speed.png".format(output_folder))

    plt.figure(3)
    plt.savefig("{}/target_angular_velocity.png".format(output_folder))

    plt.figure(4)
    plt.savefig("{}/steering_angle_percetage.png".format(output_folder))

    plt.figure(5)
    plt.savefig("{}/torque.png".format(output_folder))

    plt.figure(6)
    plt.savefig("{}/steering_angle_degree.png".format(output_folder))
def main():
    parser = argparse.ArgumentParser(description='Plot and compare reference and executed lane change trajectories.')
    parser.add_argument('--csv', type=str, required=True, help='Path to the reference trajectory CSV file.')
    parser.add_argument('--rosbag', type=str, required=True, help='Path to the executed trajectory rosbag file.')
    parser.add_argument('--output', type=str, required=True, help='Path to the folder where the plots will be saved.')
    args = parser.parse_args()

    plot_trajectory_from_csv(args.csv)
    plot_trajectory_from_rosbag(args.rosbag)

    save_plots(args.output)
    
    plt.show()

if __name__ == '__main__':
    main()
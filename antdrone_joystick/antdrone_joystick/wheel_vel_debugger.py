#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import csv
import matplotlib.pyplot as plt
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class WheelVelocitiesRecorder(Node):
    def __init__(self):
        super().__init__('wheel_velocities_recorder')

        # Robot geometry and wheel parameters (adjust as needed)
        self.lx = 0.193             # half the robot length in meters (example)
        self.ly = 0.258             # half the robot width in meters (example)
        self.wheel_radius_m = 0.0762  # wheel radius in meters (example)

        # QoS Profile for topics that use Best Effort delivery
        best_effort_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # Subscriber for /wheel_joint_states (Best Effort)
        self.subscription_states = self.create_subscription(
            JointState,
            '/wheel_joint_states',
            self.joint_state_callback,
            best_effort_qos
        )

        # Subscriber for /wheel_joint_cmds (using Best Effort here as well)
        self.subscription_cmds = self.create_subscription(
            JointState,
            '/wheel_joint_cmds',
            self.joint_cmds_callback,
            best_effort_qos
        )

        # Subscriber for /cmd_vel (Twist messages)
        self.subscription_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            best_effort_qos
        )

        # Open CSV file for wheel_joint_states velocities
        self.csv_filename_states = 'wheel_joint_states_velocities.csv'
        self.csv_file_states = open(self.csv_filename_states, mode='w', newline='')
        self.csv_writer_states = csv.writer(self.csv_file_states)
        self.csv_writer_states.writerow(['time', 'wheel_1', 'wheel_2', 'wheel_3', 'wheel_4'])

        # Open CSV file for wheel_joint_cmds velocities
        self.csv_filename_cmds = 'wheel_joint_cmds_velocities.csv'
        self.csv_file_cmds = open(self.csv_filename_cmds, mode='w', newline='')
        self.csv_writer_cmds = csv.writer(self.csv_file_cmds)
        self.csv_writer_cmds.writerow(['time', 'wheel_1', 'wheel_2', 'wheel_3', 'wheel_4'])

        # Open CSV file for cmd_vel data (recording body velocity commands)
        self.csv_filename_cmd_vel = 'cmd_vel_data.csv'
        self.csv_file_cmd_vel = open(self.csv_filename_cmd_vel, mode='w', newline='')
        self.csv_writer_cmd_vel = csv.writer(self.csv_file_cmd_vel)
        self.csv_writer_cmd_vel.writerow(['time', 'linear_x', 'linear_y', 'angular_z'])

        self.get_logger().info("Wheel Velocities Recorder Node started.")

    def joint_state_callback(self, msg: JointState):
        """
        Callback for /wheel_joint_states topic.
        Records the current time and the first four wheel velocities (if available).
        """
        now = self.get_clock().now()
        time_sec = now.nanoseconds / 1e9

        if len(msg.velocity) >= 4:
            row = [time_sec, msg.velocity[0], msg.velocity[1], msg.velocity[2], msg.velocity[3]]
            self.csv_writer_states.writerow(row)
            self.csv_file_states.flush()
        else:
            self.get_logger().warning(
                "Received JointState message on /wheel_joint_states with less than 4 velocities."
            )

    def joint_cmds_callback(self, msg: JointState):
        """
        Callback for /wheel_joint_cmds topic.
        Records the current time and the first four wheel velocities (if available).
        """
        now = self.get_clock().now()
        time_sec = now.nanoseconds / 1e9

        if len(msg.velocity) >= 4:
            row = [time_sec, msg.velocity[0], msg.velocity[1], msg.velocity[2], msg.velocity[3]]
            self.csv_writer_cmds.writerow(row)
            self.csv_file_cmds.flush()
        else:
            self.get_logger().warning(
                "Received JointState message on /wheel_joint_cmds with less than 4 velocities."
            )

    def cmd_vel_callback(self, msg: Twist):
        """
        Callback for /cmd_vel topic.
        Records the current time and the body velocity command (linear_x, linear_y, angular_z).
        """
        now = self.get_clock().now()
        time_sec = now.nanoseconds / 1e9

        row = [time_sec, msg.linear.x, msg.linear.y, msg.angular.z]
        self.csv_writer_cmd_vel.writerow(row)
        self.csv_file_cmd_vel.flush()

    def plot_data(self):
        """
        Closes CSV files, reads the saved wheel velocities data and cmd_vel data,
        computes the expected body velocities (vx, vy, omega) from the wheel velocities,
        and creates one figure with two subplots:
          - Top: Raw wheel velocities (states and wheel cmds)
          - Bottom: Computed body velocities (from wheel data) and the actual cmd_vel values.
        The figure is saved as 'wheel_velocities.png' in the current directory.
        """
        # Close CSV files to ensure all data is saved
        self.csv_file_states.close()
        self.csv_file_cmds.close()
        self.csv_file_cmd_vel.close()

        # --- Read data from /wheel_joint_states CSV file ---
        times_states = []
        fl_states = []  # front left
        fr_states = []  # front right
        bl_states = []  # back left
        br_states = []  # back right
        with open(self.csv_filename_states, 'r') as f_states:
            csv_reader = csv.reader(f_states)
            next(csv_reader)  # skip header
            for row in csv_reader:
                try:
                    times_states.append(float(row[0]))
                    fl_states.append(float(row[1]))
                    fr_states.append(float(row[2]))
                    bl_states.append(float(row[3]))
                    br_states.append(float(row[4]))
                except ValueError:
                    continue

        # --- Read data from /wheel_joint_cmds CSV file ---
        times_cmds = []
        fl_cmds = []  # front left
        fr_cmds = []  # front right
        bl_cmds = []  # back left
        br_cmds = []  # back right
        with open(self.csv_filename_cmds, 'r') as f_cmds:
            csv_reader = csv.reader(f_cmds)
            next(csv_reader)  # skip header
            for row in csv_reader:
                try:
                    times_cmds.append(float(row[0]))
                    fl_cmds.append(float(row[1]))
                    fr_cmds.append(float(row[2]))
                    bl_cmds.append(float(row[3]))
                    br_cmds.append(float(row[4]))
                except ValueError:
                    continue

        # --- Read data from /cmd_vel CSV file ---
        times_cmd_vel = []
        linear_x_cmd_vel = []
        linear_y_cmd_vel = []
        angular_z_cmd_vel = []
        with open(self.csv_filename_cmd_vel, 'r') as f_cmd_vel:
            csv_reader = csv.reader(f_cmd_vel)
            next(csv_reader)  # skip header
            for row in csv_reader:
                try:
                    times_cmd_vel.append(float(row[0]))
                    linear_x_cmd_vel.append(float(row[1]))
                    linear_y_cmd_vel.append(float(row[2]))
                    angular_z_cmd_vel.append(float(row[3]))
                except ValueError:
                    continue

        # --- Compute expected body velocities from wheel velocities ---
        # Define the inverse kinematics matrix (for a mecanum or omni-drive robot).
        inv_kin_mat = np.array([
            [1, -1, -(self.lx + self.ly)],
            [1,  1,  (self.lx + self.ly)],
            [1,  1, -(self.lx + self.ly)],
            [1, -1,  (self.lx + self.ly)]
        ])
        pseudo_inv = np.linalg.pinv(inv_kin_mat)

        # For actual (states) wheel data
        vx_states = []
        vy_states = []
        omega_states = []
        for fl, fr, bl, br in zip(fl_states, fr_states, bl_states, br_states):
            w = np.array([fl, -fr, bl, -br])
            vxytheta = np.matmul(pseudo_inv, w) * self.wheel_radius_m
            vx_states.append(vxytheta[0])
            vy_states.append(vxytheta[1])
            omega_states.append(vxytheta[2])

        # For commanded (wheel cmds) data
        vx_cmds = []
        vy_cmds = []
        omega_cmds = []
        for fl, fr, bl, br in zip(fl_cmds, fr_cmds, bl_cmds, br_cmds):
            w = np.array([fl, -fr, bl, -br])
            vxytheta = np.matmul(pseudo_inv, w) * self.wheel_radius_m
            vx_cmds.append(vxytheta[0])
            vy_cmds.append(vxytheta[1])
            omega_cmds.append(vxytheta[2])

        # --- Create a single figure with two subplots ---
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 12))

        # Top subplot: Raw wheel velocities.
        ax1.plot(times_states, fl_states, 'b-', label='FL state')
        ax1.plot(times_states, fr_states, 'g-', label='FR state')
        ax1.plot(times_states, bl_states, 'r-', label='BL state')
        ax1.plot(times_states, br_states, 'c-', label='BR state')
        ax1.plot(times_cmds, fl_cmds, 'b--', label='FL cmd')
        ax1.plot(times_cmds, fr_cmds, 'g--', label='FR cmd')
        ax1.plot(times_cmds, bl_cmds, 'r--', label='BL cmd')
        ax1.plot(times_cmds, br_cmds, 'c--', label='BR cmd')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Wheel Velocity (rad/s)')
        ax1.set_title('Raw Wheel Velocities')
        ax1.legend(loc='upper right', fontsize='small')
        ax1.grid(True)

        # Bottom subplot: Computed body velocities.
        # Plot vx, vy, and omega for states (solid lines) and wheel cmds (dashed lines)
        ax2.plot(times_states, vx_states, 'm-', label='vx state')
        ax2.plot(times_states, vy_states, 'y-', label='vy state')
        ax2.plot(times_states, omega_states, 'k-', label='omega state')
        ax2.plot(times_cmds, vx_cmds, 'm--', label='vx wheel cmd')
        ax2.plot(times_cmds, vy_cmds, 'y--', label='vy wheel cmd')
        ax2.plot(times_cmds, omega_cmds, 'k--', label='omega wheel cmd')
        # Plot cmd_vel data (direct body velocity commands) with dotted lines
        ax2.plot(times_cmd_vel, linear_x_cmd_vel, 'b:', label='vx cmd_vel')
        ax2.plot(times_cmd_vel, linear_y_cmd_vel, 'g:', label='vy cmd_vel')
        ax2.plot(times_cmd_vel, angular_z_cmd_vel, 'r:', label='omega cmd_vel')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Body Velocity (m/s or rad/s)')
        ax2.set_title('Computed Body Velocities & cmd_vel Data')
        ax2.legend(loc='upper right', fontsize='small')
        ax2.grid(True)

        plt.tight_layout()
        fig_filename = 'wheel_velocities.png'
        plt.savefig(fig_filename)
        self.get_logger().info(f"Saved plot to {fig_filename}")

def main(args=None):
    rclpy.init(args=args)
    node = WheelVelocitiesRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt detected, shutting down node...")
    finally:
        # Plot and save data from all CSV files before shutting down
        node.plot_data()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

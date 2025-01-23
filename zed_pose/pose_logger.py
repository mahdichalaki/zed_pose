import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
import csv
import os
from time import time
import math

class PoseLogger(Node):
    def __init__(self):
        super().__init__('pose_logger')

        # Declare a parameter for the file suffix
        self.declare_parameter('file_suffix', '')

        # Retrieve the suffix parameter value
        file_suffix = self.get_parameter('file_suffix').get_parameter_value().string_value

        # Define the CSV files for angles, force/torque, and odometry data with the suffix
        self.angles_csv_file = f'angles_log_{file_suffix}.csv'
        self.angles_csv_file_path = os.path.join(os.getcwd(), self.angles_csv_file)
        self.force_csv_file = f'force_log_{file_suffix}.csv'
        self.force_csv_file_path = os.path.join(os.getcwd(), self.force_csv_file)
        self.odom_csv_file = f'odom_log_{file_suffix}.csv'
        self.odom_csv_file_path = os.path.join(os.getcwd(), self.odom_csv_file)

        # Open the CSV file in write mode for angles
        self.angles_csv_file_handle = open(self.angles_csv_file_path, mode='w', newline='')
        self.angles_csv_writer = csv.writer(self.angles_csv_file_handle)

        # Write the header to the angles CSV file
        self.angles_csv_writer.writerow([
            'timestamp',
            'right_elbow_angle', 'right_torso_angle',
            'left_elbow_angle', 'left_torso_angle'
        ])

        # Open the CSV file in write mode for force/torque data
        self.force_csv_file_handle = open(self.force_csv_file_path, mode='w', newline='')
        self.force_csv_writer = csv.writer(self.force_csv_file_handle)

        # Write the header to the force/torque CSV file
        self.force_csv_writer.writerow([
            'timestamp',
            'force_x', 'force_y', 'force_z',
            'torque_x', 'torque_y', 'torque_z'
        ])

        # Open the CSV file in write mode for odometry data
        self.odom_csv_file_handle = open(self.odom_csv_file_path, mode='w', newline='')
        self.odom_csv_writer = csv.writer(self.odom_csv_file_handle)

        # Write the header to the odometry CSV file
        self.odom_csv_writer.writerow([
            'timestamp',
            'position_x', 'position_y', 'theta'  # Logging theta (yaw) instead of quaternion orientation
        ])

        # Start time reference
        self.start_time = time()

        # Buffers to store the most recent data for each topic
        self.angle_data = {
            'right_elbow_angle': None,
            'right_torso_angle': None,
            'left_elbow_angle': None,
            'left_torso_angle': None
        }
        self.force_torque_data = {
            'force_x': None,
            'force_y': None,
            'force_z': None,
            'torque_x': None,
            'torque_y': None,
            'torque_z': None
        }

        # Subscribers for the topics
        self.create_subscription(Float32MultiArray, 'angles', self.angles_callback, 10)
        self.create_subscription(WrenchStamped, '/calibrated_wrench', self.calibrated_wrench_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

    # Callback function for angles topic
    def angles_callback(self, msg):
        # Extract all angles from the Float32MultiArray message
        self.angle_data['right_elbow_angle'] = msg.data[0]
        self.angle_data['right_torso_angle'] = msg.data[1]
        self.angle_data['left_elbow_angle'] = msg.data[2]
        self.angle_data['left_torso_angle'] = msg.data[3]

        # Log the angles data
        self.log_angles_data()

    def log_angles_data(self):
        """Log the most recent angles data to the CSV file."""
        timestamp = time() - self.start_time
        row = [
            timestamp,
            self.angle_data.get('right_elbow_angle', ''), self.angle_data.get('right_torso_angle', ''),
            self.angle_data.get('left_elbow_angle', ''), self.angle_data.get('left_torso_angle', '')
        ]
        self.angles_csv_writer.writerow(row)
        self.get_logger().info(f"Angle data logged at {timestamp:.2f} seconds.")

    # Callback function for the calibrated_wrench topic (force/torque)
    def calibrated_wrench_callback(self, msg):
        # Extract force and torque from the WrenchStamped message
        try:
            self.force_torque_data['force_x'] = msg.wrench.force.x
            self.force_torque_data['force_y'] = msg.wrench.force.y
            self.force_torque_data['force_z'] = msg.wrench.force.z
            self.force_torque_data['torque_x'] = msg.wrench.torque.x
            self.force_torque_data['torque_y'] = msg.wrench.torque.y
            self.force_torque_data['torque_z'] = msg.wrench.torque.z
            self.log_force_data()
        except Exception as e:
            self.get_logger().error(f"Error extracting data from WrenchStamped message: {e}")

    def log_force_data(self):
        """Log the most recent force/torque data to the CSV file."""
        timestamp = time() - self.start_time
        row = [
            timestamp,
            self.force_torque_data.get('force_x', ''), self.force_torque_data.get('force_y', ''), self.force_torque_data.get('force_z', ''),
            self.force_torque_data.get('torque_x', ''), self.force_torque_data.get('torque_y', ''), self.force_torque_data.get('torque_z', '')
        ]
        self.force_csv_writer.writerow(row)
        self.get_logger().info(f"Force/Torque data logged at {timestamp:.2f} seconds.")

    # Callback function for the odometry topic
    def odom_callback(self, msg):
        # Extract the position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Extract the orientation in quaternion format and convert to yaw (theta)
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        theta = math.atan2(siny_cosp, cosy_cosp)

        # Log the odometry data
        self.log_odom_data(x, y, theta)

    def log_odom_data(self, x, y, theta):
        """Log the most recent odometry data to the CSV file."""
        timestamp = time() - self.start_time
        row = [timestamp, x, y, theta]
        self.odom_csv_writer.writerow(row)
        self.get_logger().info(f"Odometry data logged at {timestamp:.2f} seconds.")

    def destroy_node(self):
        # Close the CSV files when the node is destroyed
        self.angles_csv_file_handle.close()
        self.force_csv_file_handle.close()
        self.odom_csv_file_handle.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    pose_logger = PoseLogger()
    rclpy.spin(pose_logger)
    pose_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

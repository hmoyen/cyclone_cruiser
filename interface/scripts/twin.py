rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Twist
import math
import numpy as np
import time

# Initial position and orientation
x, y, theta = 0, 0, 0  # Initial position (x, y) in meters and theta in degrees
x_sim, y_sim, theta_sim = 0, 0, 0  # Initial position (x, y) in meters and theta in degrees

# Constants for real robot
wheel_radius = 0.0325  # meters
wheel_base = 0.1275    # meters

wheel_radius_sim = 0.4  # meters
wheel_base_sim = 1.2    # meters

class RotationListener(Node):
    def __init__(self):
        super().__init__('rotation_listener')

        # Subscribers for rotation data
        self.subscription = self.create_subscription(
            String,
            '/rotations',
            self.listener_callback,
            10
        )

        # Publisher for robot velocity
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher for path
        self.path_publisher = self.create_publisher(Path, '/robot_path', 10)

        # Initialize the path message
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"

        # Timer to stop movement after some time
        self.timer = None
        self.start_time = None
        self.movement_duration = 0.1  # Seconds

    def listener_callback(self, msg):
        global x, y, theta, x_sim, y_sim, theta_sim

        # Parse the message, e.g., "R:1;L:0"
        rotation_data = msg.data.split(";")
        delta_right = int(rotation_data[0].split(":")[1])
        delta_left = int(rotation_data[1].split(":")[1])

        # Calculate distances moved by each wheel (scaled)
        d_right = wheel_radius * 2 * (np.pi / 15) * delta_right 
        d_left = wheel_radius * 2 * (np.pi / 15) * delta_left 

        # Calculate orientation change and average distance (scaled)
        delta_theta = (d_right - d_left) / (2 * wheel_base )
        d = (d_right + d_left) / 2

        # Update position and orientation
        x += d * math.cos(math.radians(theta) + delta_theta / 2)
        y += d * math.sin(math.radians(theta) + delta_theta / 2)
        theta += math.degrees(delta_theta)
        theta = (theta + 180) % 360 - 180  # Normalize to [-180, 180]

        # Add current position to path
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(math.radians(theta) / 2)
        pose.pose.orientation.w = math.cos(math.radians(theta) / 2)
        self.path_msg.poses.append(pose)
        self.path_msg.header.stamp = self.get_clock().now().to_msg()

        self.path_publisher.publish(self.path_msg)

        # Calculate distances moved by each wheel (scaled)
        d_right_sim = wheel_radius_sim * 2 * (np.pi / 20) * delta_right 
        d_left_sim = wheel_radius_sim * 2 * (np.pi / 20) * delta_left 

        # Calculate orientation change and average distance (scaled)
        delta_theta_sim = (d_right_sim - d_left_sim) / (2 * wheel_base_sim)
        d_sim = (d_right_sim + d_left_sim) / 2

        # Update position and orientation
        x_sim += d_sim * math.cos(math.radians(theta_sim) + delta_theta_sim / 2)
        y_sim += d_sim * math.sin(math.radians(theta_sim) + delta_theta_sim/ 2)
        theta_sim += math.degrees(delta_theta_sim)
        theta_sim = (theta_sim + 180) % 360 - 180  # Normalize to [-180, 180]

    
        # Call move function to control the robot
        self.move_robot(delta_x=d_sim, delta_y=0, delta_theta=delta_theta_sim)

    def move_robot(self, delta_x, delta_y, delta_theta):
        # Calculate linear and angular velocities to achieve delta_x, delta_y, delta_theta
        velocity = Twist()

        # Linear velocity scaled by SCALE_WHEEL_RADIUS to match digital twin
        linear_velocity = (math.sqrt(delta_x**2 + delta_y**2) / self.movement_duration) 
        angular_velocity = (delta_theta / self.movement_duration) 

        # Set linear and angular velocities
        velocity.linear.x = linear_velocity
        velocity.angular.z = angular_velocity

        # Publish the velocities to /cmd_vel
        self.velocity_publisher.publish(velocity)

        # Stop movement after a given time period (2 seconds)
        self.start_time = self.get_clock().now().to_msg()
        self.timer = self.create_timer(self.movement_duration, self.stop_robot)

    def stop_robot(self):
        # Stop the robot by publishing zero velocities
        stop_velocity = Twist()
        self.velocity_publisher.publish(stop_velocity)
        
        # Destroy the timer
        if self.timer:
            self.timer.cancel()
            self.timer = None

def main(args=None):
    rclpy.init(args=args)
    rotation_listener = RotationListener()

    try:
        rclpy.spin(rotation_listener)
    except KeyboardInterrupt:
        pass

    rotation_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
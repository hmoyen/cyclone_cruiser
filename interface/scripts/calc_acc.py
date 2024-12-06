import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist
from visualization_msgs.msg import Marker, MarkerArray
import math
import numpy as np

# Constants for the real robot
wheel_radius = 0.0325  # meters
wheel_base = 0.1275 / 2  # meters

class RotationListener(Node):
    def __init__(self):
        super().__init__('rotation_listener')

        # Subscribers
        self.rotation_subscriber = self.create_subscription(
            String,
            '/rotations/total',
            self.listener_callback,
            10
        )
        self.sonar1_subscriber = self.create_subscription(
            String,
            '/sonar1',
            self.sonar1_callback,
            1
        )
        self.sonar2_subscriber = self.create_subscription(
            String,
            '/sonar2',
            self.sonar2_callback,
            1
        )

        # Publishers
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_publisher = self.create_publisher(Path, '/robot_path', 10)
        self.marker_array_publisher = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Orientation in degrees
        self.prev_cw_right = 0
        self.prev_ccw_right = 0
        self.prev_cw_left = 0
        self.prev_ccw_left = 0

        # List to store sonar points
        self.sonar_points = []  # Initialize sonar_points as an empty list

        # Initialize the path message
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"


    def listener_callback(self, msg):
        # Parse the message, e.g., "RCCW:78;RCW:76;LCCW:75;LCW:34"
        try:
            data = dict(item.split(":") for item in msg.data.split(";"))
            cw_right = int(data["LCW"])
            ccw_right = int(data["LCCW"])
            cw_left = int(data["RCW"])
            ccw_left = int(data["RCCW"])
        except (KeyError, ValueError):
            self.get_logger().error("Invalid rotation message format")
            return

        # Calculate delta rotations
        delta_cw_right = cw_right - self.prev_cw_right
        delta_ccw_right = ccw_right - self.prev_ccw_right
        delta_cw_left = cw_left - self.prev_cw_left
        delta_ccw_left = ccw_left - self.prev_ccw_left

        # Update previous counts
        self.prev_cw_right = cw_right
        self.prev_ccw_right = ccw_right
        self.prev_cw_left = cw_left
        self.prev_ccw_left = ccw_left

        # Calculate net rotations
        delta_right = delta_cw_right - delta_ccw_right
        delta_left = delta_cw_left - delta_ccw_left

        # Calculate distances moved by each wheel
        d_right = wheel_radius * 2 * (np.pi / 20) * delta_right
        d_left = wheel_radius * 2 * (np.pi / 20) * delta_left

        # Calculate orientation change and average distance
        delta_theta = (d_right - d_left) / (2 * wheel_base)
        d = (d_right + d_left) / 2

        # Update position and orientation
        self.x += d * math.cos(math.radians(self.theta) + delta_theta / 2)
        self.y += d * math.sin(math.radians(self.theta) + delta_theta / 2)
        self.theta += math.degrees(delta_theta)
        self.theta = (self.theta + 180) % 360 - 180  # Normalize to [-180, 180]

        # Add current position to path
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.orientation.z = math.sin(math.radians(self.theta) / 2)
        pose.pose.orientation.w = math.cos(math.radians(self.theta) / 2)
        self.path_msg.poses.append(pose)
        self.path_msg.header.stamp = self.get_clock().now().to_msg()

        self.path_publisher.publish(self.path_msg)

    def sonar1_callback(self, msg):
        self.add_sonar_point(msg, angle_offset=-90.0, sonar_id=1)
        self.get_logger().info(f"Sonar1 message received: {msg.data}")



    def sonar2_callback(self, msg):
        self.add_sonar_point(msg, angle_offset=90.0, sonar_id=2)
        self.get_logger().info(f"Sonar2 message received: {msg.data}")

    def add_sonar_point(self, msg, angle_offset, sonar_id):
        # Parse the distance from the message
        try:
            distance_cm = int(msg.data.split(":")[1])
            distance_m = distance_cm / 100.0
        except (IndexError, ValueError):
            self.get_logger().error("Invalid sonar message format")
            return

        # Calculate the sonar point relative to the robot
        angle_rad = math.radians(self.theta + angle_offset)
        sonar_x = self.x + distance_m * math.cos(angle_rad)
        sonar_y = self.y + distance_m * math.sin(angle_rad)

        # Append the point and sonar_id to the list
        self.sonar_points.append((sonar_x, sonar_y, sonar_id))

        # Publish the updated marker array
        self.publish_marker_array()

    def publish_marker_array(self):
        marker_array = MarkerArray()

        for i, (sonar_x, sonar_y, sonar_id) in enumerate(self.sonar_points):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "sonar_points"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = sonar_x
            marker.pose.position.y = sonar_y
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.07
            marker.scale.y = 0.07
            marker.scale.z = 0.07
            marker.color.a = 1.0

            # Assign different colors based on sonar_id
            if sonar_id == 1:
                marker.color.r = 1.0  # Red for sonar1
                marker.color.g = 0.0
                marker.color.b = 0.0
            elif sonar_id == 2:
                marker.color.r = 0.0
                marker.color.g = 0.0  # Blue for sonar2
                marker.color.b = 1.0

            marker_array.markers.append(marker)

        self.marker_array_publisher.publish(marker_array)


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

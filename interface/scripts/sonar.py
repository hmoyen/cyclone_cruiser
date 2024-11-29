import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import math

class RotationListener(Node):
    def __init__(self):
        super().__init__('rotation_listener')

        # Subscribers for sonar data
        self.sonar1_sub = self.create_subscription(
            String,
            '/sonar1',
            self.sonar1_callback,
            10
        )
        self.sonar2_sub = self.create_subscription(
            String,
            '/sonar2',
            self.sonar2_callback,
            10
        )

        # Publisher for markers
        self.marker_array_publisher = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)

        # Robot position and orientation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Orientation in degrees

        # List to store sonar points
        self.sonar_points = []

    def sonar1_callback(self, msg):
        self.add_sonar_point(msg, angle_offset=90.0, marker_id=len(self.sonar_points))

    def sonar2_callback(self, msg):
        self.add_sonar_point(msg, angle_offset=-90.0, marker_id=len(self.sonar_points))

    def add_sonar_point(self, msg, angle_offset, marker_id):
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

        # Append the point to the list
        self.sonar_points.append((sonar_x, sonar_y))

        # Publish the updated marker array
        self.publish_marker_array()

    def publish_marker_array(self):
        marker_array = MarkerArray()

        for i, (sonar_x, sonar_y) in enumerate(self.sonar_points):
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
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

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

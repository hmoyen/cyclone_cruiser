import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import math

class SimulatedPublisher(Node):
    def __init__(self):
        super().__init__('simulated_publisher')

        # Publishers for rotation and sonar topics
        self.rotation_publisher = self.create_publisher(String, '/rotations', 10)
        self.sonar1_publisher = self.create_publisher(String, '/sonar1', 10)
        self.sonar2_publisher = self.create_publisher(String, '/sonar2', 10)

        # Timer to publish data
        self.timer = self.create_timer(0.5, self.publish_data)

        # Simulated robot state
        self.x = 0.0  # X position
        self.y = 0.0  # Y position
        self.theta = 0.0  # Orientation in degrees
        self.speed = 0.1  # Speed in meters per second
        self.distance_between_features = 2.0  # Distance between features in meters

        # Track features for sonar
        self.features = [
            (2.0, 2.0),  # Feature near sonar1
            (6.0, -2.0),  # Feature near sonar2
            (10.0, 2.0),  # Another feature near sonar1
        ]
        self.current_time = 0.0

    def publish_data(self):
        # Publish rotation data
        delta_right = 5  # Simulate 5 ticks for the right wheel
        delta_left = 5   # Simulate 5 ticks for the left wheel
        rotation_msg = f"R:{delta_right};L:{delta_left}"
        self.rotation_publisher.publish(String(data=rotation_msg))

        # Update robot state
        d = self.speed * 0.5  # Distance traveled in 0.5 seconds
        self.x += d * math.cos(math.radians(self.theta))
        self.y += d * math.sin(math.radians(self.theta))

        # Publish sonar data
        sonar1_distance = self.get_sonar_distance(self.features, 90.0)
        sonar2_distance = self.get_sonar_distance(self.features, -90.0)

        if sonar1_distance:
            sonar1_msg = f"DISTANCE:{int(sonar1_distance * 100)}"  # Convert to cm
            self.sonar1_publisher.publish(String(data=sonar1_msg))

        if sonar2_distance:
            sonar2_msg = f"DISTANCE:{int(sonar2_distance * 100)}"  # Convert to cm
            self.sonar2_publisher.publish(String(data=sonar2_msg))

    def get_sonar_distance(self, features, angle_offset):
        """
        Calculate the distance to the nearest feature in the sonar's direction.
        """
        angle_rad = math.radians(self.theta + angle_offset)
        sonar_x = self.x + math.cos(angle_rad)
        sonar_y = self.y + math.sin(angle_rad)

        min_distance = None
        for feature_x, feature_y in self.features:
            distance = math.sqrt((feature_x - sonar_x)**2 + (feature_y - sonar_y)**2)
            if min_distance is None or distance < min_distance:
                min_distance = distance

        # Return None if no features are within 3 meters
        return min_distance if min_distance and min_distance <= 3.0 else None


def main(args=None):
    rclpy.init(args=args)
    simulated_publisher = SimulatedPublisher()

    try:
        rclpy.spin(simulated_publisher)
    except KeyboardInterrupt:
        pass

    simulated_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math


class IntegratedSimulatedPublisher(Node):
    def __init__(self):
        super().__init__('integrated_simulated_publisher')

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
        self.speed = 0.2  # Speed in meters per second

        # Sequence control
        self.sequence = 0  # 0: Straight, 1: Turning, 2: Returning
        self.straight_distance = 2.0  # Move straight for 2 meters
        self.turn_angle = 90.0  # Turn 90 degrees
        self.turning_progress = 0.0  # Tracks how much of the turn is completed

        # Shutdown flag
        self.shutdown_flag = False

    def publish_data(self):
        if self.shutdown_flag:
            return  # Stop all activity after shutdown is initiated

        if self.sequence == 0:
            # Move straight
            self.move_straight()
            if self.distance_traveled() >= self.straight_distance:
                self.sequence = 1  # Switch to turning
                self.turning_progress = 0.0

        elif self.sequence == 1:
            # Turn 90 degrees
            self.turn()
            if abs(self.turning_progress) >= abs(self.turn_angle):
                self.sequence = 2  # Switch to returning

        elif self.sequence == 2:
            # Move back to the starting point
            self.move_back_to_start()
            if self.distance_to_start() < 0.1:
                self.get_logger().info("Returned to starting point")
                self.shutdown_flag = True
                self.timer.cancel()  # Stop the timer
                self.destroy_node()
                rclpy.shutdown()

        # Publish sonar and rotation data
        self.publish_rotation_data()
        self.publish_sonar_data()

    def move_straight(self):
        d = self.speed * 0.5  # Distance traveled in 0.5 seconds
        self.x += d * math.cos(math.radians(self.theta))
        self.y += d * math.sin(math.radians(self.theta))

    def turn(self):
        angular_speed = 30.0  # Degrees per second
        delta_theta = angular_speed * 0.5  # Change in orientation in 0.5 seconds
        self.theta += delta_theta
        self.turning_progress += delta_theta

        # Wrap theta to stay within [-180, 180]
        self.theta = (self.theta + 180) % 360 - 180

    def move_back_to_start(self):
        angle_to_start = math.degrees(math.atan2(-self.y, -self.x))
        self.theta = angle_to_start
        self.move_straight()

    def distance_traveled(self):
        return math.sqrt(self.x**2 + self.y**2)

    def distance_to_start(self):
        return math.sqrt(self.x**2 + self.y**2)

    def publish_rotation_data(self):
        delta_right = 10  # Simulated encoder ticks
        delta_left = 10
        if self.sequence == 1:  # Turning
            delta_right += 5
            delta_left -= 5
        rotation_msg = f"R:{delta_right};L:{delta_left}"
        self.rotation_publisher.publish(String(data=rotation_msg))

    def publish_sonar_data(self):
        # Simulate sonar readings based on orientation
        left_distance = 0.5  # Default left sonar distance (meters)
        right_distance = 0.5  # Default right sonar distance (meters)

        # Adjust the distance based on the robot's orientation
        if -45 <= self.theta <= 45:
            # Robot facing forward, so sonar readings are constant
            left_distance = 0.5
            right_distance = 0.5
        elif 45 < self.theta <= 135:
            # Robot facing to the right
            left_distance = 0.3
            right_distance = 0.7
        elif 135 < self.theta <= 180 or -180 <= self.theta < -135:
            # Robot facing backward
            left_distance = 0.7
            right_distance = 0.3
        elif -135 < self.theta <= -45:
            # Robot facing to the left
            left_distance = 0.3
            right_distance = 0.7

        # Publish the sonar distances (converted to centimeters)
        sonar1_msg = f"D:{int(left_distance * 100)}"  # Convert to cm
        sonar2_msg = f"D:{int(right_distance * 100)}"  # Convert to cm
        self.sonar1_publisher.publish(String(data=sonar1_msg))
        self.sonar2_publisher.publish(String(data=sonar2_msg))


def main(args=None):
    rclpy.init(args=args)
    integrated_simulated_publisher = IntegratedSimulatedPublisher()

    try:
        rclpy.spin(integrated_simulated_publisher)
    except KeyboardInterrupt:
        pass

    integrated_simulated_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

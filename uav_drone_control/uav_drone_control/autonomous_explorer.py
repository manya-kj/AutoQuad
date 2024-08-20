import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
from enum import Enum

class State(Enum):
    FORWARD = 1
    TURNING = 2
    WALL_FOLLOWING = 3
    EXPLORE = 4

class AutonomousExplorer(Node):
    def __init__(self):
        super().__init__('autonomous_explorer')
        self.publisher_ = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, '/simple_drone/scan', self.laser_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/simple_drone/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.03, self.move_drone)  # Increased update rate
        
        self.laser_data = None
        self.position = None
        self.orientation = None
        
        self.min_distance = 0.3  # Reduced minimum distance
        self.linear_speed = 20.0  # Increased linear speed
        self.angular_speed = 8.5  # Increased angular speed
        
        self.state = State.FORWARD
        self.wall_follow_direction = 1  # 1 for right, -1 for left
        self.turn_direction = 1  # 1 for right, -1 for left
        self.state_timer = 0
        self.wall_follow_timer = 0
        self.turn_count = 0
        self.stuck_timer = 0
        self.explored_area = set()
        self.grid_size = 0.5  # Size of each grid cell for tracking explored area
        self.last_position = None
        self.stuck_threshold = 1.0  # Distance threshold to consider the drone stuck

    def laser_callback(self, msg):
        self.laser_data = msg.ranges

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
        self.update_explored_area()
        self.check_if_stuck()

    def update_explored_area(self):
        x = round(self.position.x / self.grid_size) * self.grid_size
        y = round(self.position.y / self.grid_size) * self.grid_size
        self.explored_area.add((x, y))

    def check_if_stuck(self):
        if self.last_position:
            distance = ((self.position.x - self.last_position.x)**2 + 
                        (self.position.y - self.last_position.y)**2)**0.5
            if distance < self.stuck_threshold:
                self.stuck_timer += 1
            else:
                self.stuck_timer = 0
        self.last_position = self.position

    def move_drone(self):
        if self.laser_data is None:
            self.get_logger().warn('No laser data received yet')
            return

        twist = Twist()
        
        front_distance = min(self.laser_data[0:30] + self.laser_data[330:])
        left_distance = min(self.laser_data[60:120])
        right_distance = min(self.laser_data[240:300])

        if self.stuck_timer > 50:  # If stuck for too long, change state to EXPLORE
            self.state = State.EXPLORE
            self.stuck_timer = 0

        if self.state == State.FORWARD:
            if front_distance < self.min_distance:
                self.state = State.TURNING
                self.state_timer = 8  # Reduced turning time
                self.turn_direction = 1 if left_distance > right_distance else -1
            else:
                twist.linear.x = self.linear_speed
                twist.linear.z = 0.1  # Add a small upward velocity to maintain altitude
        
        elif self.state == State.TURNING:
            twist.angular.z = self.angular_speed * self.turn_direction
            self.state_timer -= 1
            if self.state_timer <= 0:
                self.state = State.WALL_FOLLOWING
                self.wall_follow_direction = -self.turn_direction  # Follow the opposite wall
                self.wall_follow_timer = 40  # Reduced wall following time
                self.turn_count += 1

        elif self.state == State.WALL_FOLLOWING:
            if self.wall_follow_direction == 1:  # Following right wall
                error = right_distance - self.min_distance
            else:  # Following left wall
                error = self.min_distance - left_distance
            
            twist.linear.x = self.linear_speed * 0.9  # Slightly reduced speed during wall following
            twist.angular.z = error * 1.2  # Increased proportional control
            
            self.wall_follow_timer -= 1
            if self.wall_follow_timer <= 0 or front_distance < self.min_distance:
                self.state = State.TURNING
                self.state_timer = 8
                self.turn_direction = self.wall_follow_direction

        elif self.state == State.EXPLORE:
            unexplored_direction = self.find_unexplored_direction()
            twist.linear.x = self.linear_speed * 0.7
            twist.angular.z = self.angular_speed * 0.5 * unexplored_direction
            if front_distance < self.min_distance:
                self.state = State.TURNING
                self.state_timer = 8
                self.turn_direction = unexplored_direction

        # Implement a more aggressive coverage strategy
        if len(self.explored_area) > 300 or self.turn_count >= 10:  # Increased threshold
            self.state = State.EXPLORE
            self.turn_count = 0
            self.explored_area.clear()  # Reset explored area

        self.publisher_.publish(twist)
        self.get_logger().info(f'State: {self.state}, Front: {front_distance:.2f}, Left: {left_distance:.2f}, Right: {right_distance:.2f}')
        self.get_logger().info(f'Sending velocity command: linear_x={twist.linear.x:.2f}, angular_z={twist.angular.z:.2f}')
        self.get_logger().info(f'Explored area size: {len(self.explored_area)}')

    def find_unexplored_direction(self):
        # Improved method to find an unexplored direction
        left_sum = sum(self.laser_data[60:120])
        right_sum = sum(self.laser_data[240:300])
        front_sum = sum(self.laser_data[0:30] + self.laser_data[330:])
        
        if front_sum > max(left_sum, right_sum):
            return 0  # Go straight
        elif left_sum > right_sum:
            return -1  # Turn left
        else:
            return 1  # Turn right

def main(args=None):
    rclpy.init(args=args)
    autonomous_explorer = AutonomousExplorer()
    rclpy.spin(autonomous_explorer)
    autonomous_explorer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

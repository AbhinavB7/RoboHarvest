#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class KeyboardControlNode(Node):

    # Define the constructor of the class
    def __init__(self):
        super().__init__('keyboard_control_node')
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.wheel_base = 0.8382
        self.wheel_rad = 0.25224 / 2.0
        self.cmd_vel = Twist()
        self.target_x = 6.5  # Target x-coordinate
        self.in_motion = False # Flag to indicate if the robot is in motion
        self.duration = 6.0  
        self.timer = self.create_timer(self.duration, self.timer_callback)

    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg
        self.get_logger().info('I heard: "%s"' % msg)
        self.calc_vel()

    def calc_vel(self):
        linear_vel = self.cmd_vel.linear.x
        angular_vel = self.cmd_vel.angular.z

        # Calculate wheel velocities
        left_vel = (linear_vel - angular_vel * self.wheel_base / 2.0) / self.wheel_rad
        right_vel = (linear_vel + angular_vel * self.wheel_base / 2.0) / self.wheel_rad

        wheel_velocities = Float64MultiArray()
        wheel_velocities.data = [-right_vel, -right_vel, left_vel, left_vel]

        self.wheel_velocities_pub.publish(wheel_velocities)

        # Check if the robot has reached the target x-coordinate
        if not self.in_motion and abs(self.target_x) < 0.1:
            # Stop the robot
            wheel_velocities.data = [0.0, 0.0, 0.0, 0.0]
            self.wheel_velocities_pub.publish(wheel_velocities)
            self.get_logger().info('Reached target x-coordinate: {}'.format(self.target_x))
            self.in_motion = True

    def timer_callback(self):
        # Stop the robot after the specified duration
        if not self.in_motion:
            wheel_velocities = Float64MultiArray()
            wheel_velocities.data = [0.0, 0.0, 0.0, 0.0]
            self.wheel_velocities_pub.publish(wheel_velocities)
            self.get_logger().info('Stopped after {} seconds'.format(self.duration))
            self.in_motion = True

        
    
    
def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()

    try:
        # Publish a non-zero cmd_vel to trigger robot movement
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.5
        cmd_vel.angular.z = 0.0
        node.cmd_vel_callback(cmd_vel)

        # Spin until the timer callback stops the robot
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

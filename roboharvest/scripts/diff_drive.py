#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import sys
import select
import tty
import termios
from pynput import keyboard

class KeyboardControlNode(Node):

    def __init__(self):
        super().__init__('keyboard_control_node')
        self.wheel_velocities_pub = self.create_publisher(
            Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_state_pub = self.create_publisher(
            JointState, '/joint_states', 10)
        self.sub_cmd_vel = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.wheel_base = 0.8382
        self.wheel_rad = 0.25224/2.0
        
        self.settings = termios.tcgetattr(sys.stdin)

    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg
        self.get_logger().info('I heard: "%s"' % msg)
        self.calc_vel()
        
    def calc_vel(self):
        self.linear_vel = self.cmd_vel.linear.x
        self.angular_vel = self.cmd_vel.angular.z
        self.left_vel = (self.linear_vel - self.angular_vel*self.wheel_base/2.0)/self.wheel_rad
        self.right_vel = (self.linear_vel + self.angular_vel*self.wheel_base/2.0)/self.wheel_rad

        wheel_velocities = Float64MultiArray()
    
        wheel_velocities.data = [-self.right_vel, -self.right_vel, self.left_vel, self.left_vel]

        self.wheel_velocities_pub.publish(wheel_velocities)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    



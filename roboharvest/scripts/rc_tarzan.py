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

# Define key codes
LIN_VEL_STEP_SIZE = 6
LEFT_STEP_SIZE = 3
RIGHT_STEP_SIZE = 2

class KeyboardControlNode(Node):

    def __init__(self):
        super().__init__('keyboard_control_node')
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run_keyboard_control(self):
        self.msg = """
        Control Your Car!
        ---------------------------
        Moving around:
            w
        a    s    d

        q : force stop

        Esc to quit
        """

        self.get_logger().info(self.msg)
        wheel_velocities = Float64MultiArray()
        linear_vel=0.0
        right_vel=0.0
        left_vel=0.0

        while True:
            key = self.getKey()
            if key is not None:
                if key == '\x1b':  # Escape key
                    break
                elif key == 'q':  # Quit
                    linear_vel=0.0
                    right_vel=0.0
                    left_vel=0.0
                elif key == 'w':  # Forward
                    linear_vel += LIN_VEL_STEP_SIZE
                elif key == 's':  # Reverse
                    linear_vel -= LIN_VEL_STEP_SIZE
                elif key == 'd':  # Right
                    right_vel -= RIGHT_STEP_SIZE
                    left_vel += LEFT_STEP_SIZE
                elif key == 'a':  # Left
                    left_vel -= LEFT_STEP_SIZE
                    right_vel += RIGHT_STEP_SIZE

                print("Linear Velocity",linear_vel)
                print("Right Turn",right_vel)
                print("Left Turn", left_vel)
                # Publish the twist message
                wheel_velocities.data = [-(linear_vel+(right_vel)),-(linear_vel+(right_vel)),linear_vel+(left_vel),linear_vel+(left_vel)]

                self.wheel_velocities_pub.publish(wheel_velocities)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    node.run_keyboard_control()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
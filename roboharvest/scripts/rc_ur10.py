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
ANG_VEL_STEP_SIZE = 0.1

class KeyboardControlNode(Node):

    def __init__(self):
        super().__init__('keyboard_control_node')

        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
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
        Control Your UR10!
        ---------------------------
        Adding value: J1 = a, J2 = s, J3 = d, J4 = f, J5 = g, J6 = h, J7 = j
        Removing value: J1 = z, J2 = x, J3 = c, J4 = v, J5 = b, J6 = n, J7 = m
        
        q : default position

        Esc to quit
        """

        self.get_logger().info(self.msg)
        joint_positions = Float64MultiArray()
        J1 = 0.0
        J2 = 0.0
        J3 = 0.0
        J4 = 0.0
        J5 = 0.0
        J6 = 0.0         
        J7 = 0.0  

        while True:
            key = self.getKey()
            if key is not None:
                if key == '\x1b':  # Escape key
                    break
                elif key == 'q':  # Home Position
                    J1 = 0.0
                    J2 = 0.0
                    J3 = 0.0
                    J4 = 0.0
                    J5 = 0.0
                    J6 = 0.0         
                    J7 = 0.0                                   
                elif key == 'a': # J1 +
                    J1 += ANG_VEL_STEP_SIZE
                elif key == 'z': # J1 -
                    J1 -= ANG_VEL_STEP_SIZE
                elif key == 's':  # J2 +
                    J2 += ANG_VEL_STEP_SIZE
                elif key == 'x':  # J2 -
                    J2 -= ANG_VEL_STEP_SIZE
                elif key == 'd':  # J3 +
                    J3 += ANG_VEL_STEP_SIZE
                elif key == 'c':  # J3 -
                    J3 -= ANG_VEL_STEP_SIZE
                elif key == 'f':  # J4 +
                    J4 += ANG_VEL_STEP_SIZE
                elif key == 'v':  # J4 -
                    J4 -= ANG_VEL_STEP_SIZE
                elif key == 'g':  # J5 +
                    J5 += ANG_VEL_STEP_SIZE
                elif key == 'b':  # J5 -
                    J5 -= ANG_VEL_STEP_SIZE
                elif key == 'h':  # J6 +
                    J6 += ANG_VEL_STEP_SIZE
                elif key == 'n':  # J6 -
                    J6 -= ANG_VEL_STEP_SIZE
                elif key == 'j':  # J7 +
                    J7 += ANG_VEL_STEP_SIZE
                elif key == 'm':  # J7 -
                    J7 -= ANG_VEL_STEP_SIZE
                
                print("J1",J1)
                print("J2",J2)
                print("J3",J3)
                print("J4",J4)
                print("J5",J5)
                print("J6",J6)
                print("J7",J7)    
                # Publish the twist message
                joint_positions.data = [J1,J2,J3,J4,J5,J6,J7]
                self.joint_position_pub.publish(joint_positions)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    node.run_keyboard_control()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
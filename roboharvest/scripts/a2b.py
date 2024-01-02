#!/usr/bin/env python3

# Import necessary libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from geometry_msgs.msg import Vector3
import matplotlib.pyplot as plt
from sympy import *
import numpy as np
import math
from mpl_toolkits.mplot3d import Axes3D
import time

# Define RobotControl class which inherits from Node
class RobotControl(Node):
    def __init__(self):
        # Initialize the node with the name 'robot_controller'
        super().__init__('robot_controller')
        
        # Create a publisher for joint positions
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,history=HistoryPolicy.KEEP_LAST,depth=10)
        self.timer = self.create_timer(1, self.publish_joint_positions)
        self.end_goal = False
        
        # Set the goal position and home position of the arm
        self.goal_position = [-100.0, 1200.0, 800.5, 0.0, 0.0, 0.0]
        self.home_pos = [0.0, 356, 1428, 0.0, 0.0, 0.0]
        self.inv_kin()
        
    # Define function to create transformation matrix
    def Transformation_matrix(self, a, al, d, th):
        return Matrix([
            [cos(th), -sin(th) * cos(al), sin(th) * sin(al), a * cos(th)],
            [sin(th), cos(th) * cos(al), -cos(th) * sin(al), a * sin(th)],
            [0, sin(al), cos(al), d],
            [0, 0, 0, 1]
        ])
        
    # Define function to calculate Jacobian
    def jacobian(self, th1, th2, th3, th4, th5, th6):
        T0 = self.Transformation_matrix(0, rad(0), 0, rad(0))
        T1 = self.Transformation_matrix(0, -rad(90), 128, th1)
        T2 = self.Transformation_matrix(-612.7, rad(180), 0, th2 + rad(90))
        T3 = self.Transformation_matrix(-571.6, rad(180), 0, th3)
        T4 = self.Transformation_matrix(0, rad(90), 163.9, th4 - rad(90))
        T5 = self.Transformation_matrix(0, -rad(90), 115.7, th5)
        T6 = self.Transformation_matrix(0, rad(0), 192.2, th6)
        T01 = (T1)
        T02 = (T01 * T2)
        T03 = (T02 * T3)
        T04 = (T03 * T4)
        T05 = (T04 * T5)
        T06 = (T05 * T6)

        z0 = Matrix(T0[:, 2][:3])
        z1 = Matrix(T01[:, 2][:3])
        z2 = Matrix(T02[:, 2][:3])
        z3 = Matrix(T03[:, 2][:3])
        z4 = Matrix(T04[:, 2][:3])
        z5 = Matrix(T05[:, 2][:3])
        z6 = Matrix(T06[:, 2][:3])

        o0 = Matrix(T0[:, 3][:3])
        o1 = Matrix(T01[:, 3][:3])
        o2 = Matrix(T02[:, 3][:3])
        o3 = Matrix(T03[:, 3][:3])
        o4 = Matrix(T04[:, 3][:3])
        o5 = Matrix(T05[:, 3][:3])
        o6 = Matrix(T06[:, 3][:3])

        J1 = Matrix([z0.cross(o6 - o0), z0])
        J2 = Matrix([z1.cross(o6 - o1), z1])
        J3 = Matrix([z2.cross(o6 - o2), z2])
        J4 = Matrix([z3.cross(o6 - o3), z3])
        J5 = Matrix([z4.cross(o6 - o4), z4])
        J6 = Matrix([z5.cross(o6 - o5), z5])
        J = J1.row_join(J2).row_join(J3).row_join(
            J4).row_join(J5).row_join(J6).applyfunc(lambda x: sympify(x))
        return J, T0, T01, T02, T03, T04, T05, T06

    # Define function to calculate inverse kinematics
    def inv_kin(self):
        J, T0, T01, T02, T03, T04, T05, T06 = self.jacobian(0.001, 0.001, 0.001, 0.001, 0.001, 0.001)
        
        damping_factor = 0.01
        J_float = np.matrix(J).astype(np.float64)
        J_t = J_float.T
        J_dinv = (J_t)*((J_float*J_t) + (damping_factor*damping_factor*np.eye(6)))
        damping_factor = 0.01

        dt = 0.1
        Time = 200
        i = 0
        trajectory_points = []
    
        q = Matrix([0, 0, 0, 0, 0, 0])
        x_dot = ((self.goal_position[0] - self.home_pos[0]) / Time)
        y_dot = ((self.goal_position[1] - self.home_pos[1]) / Time)
        z_dot = ((self.goal_position[2] - self.home_pos[2]) / Time)
        wx = (self.goal_position[3] - self.home_pos[3]) / Time
        wy = (self.goal_position[4] - self.home_pos[4]) / Time
        wz = (self.goal_position[5] - self.home_pos[5]) / Time

        # loop to calculate joint positions
        while (i < Time and not self.end_goal):
            E = Matrix([x_dot, y_dot, z_dot, wx, wy, wz])
            J_dinv = np.dot(J_t, np.linalg.inv(np.dot(J_float, J_t) + damping_factor**2 * np.eye(6)))
            q_dot = J_dinv.dot(E)
            q += q_dot * dt

            [q1, q2, q3, q4, q5, q6] = [float(q[i]) for i in range(6)]

            J, T0, T01, T02, T03, T04, T05, T06 = self.jacobian(q1, q2, q3, q4, q5, q6)
            J_float = np.matrix(J).astype(np.float64)

            T = T06
            x = T[0, 3]
            y = T[1, 3]
            z = T[2, 3]
            trajectory_points.append((x, y, z))
            print("x ", x, "y ", y, "z ", z)
            i = i + dt
            i = round(i, 3)

            msg = Float64MultiArray()
            msg.data = [q1, q2, q3, q4, q5, q6]

            self.joint_position_pub.publish(msg)

            if abs(x - self.goal_position[0]) < 0.01 and abs(y - self.goal_position[1]) < 0.01 and abs(z - self.goal_position[2]) < 0.01:
                print("Goal reached!")
                self.end_goal = True

           
        status=False                   

                
    def publish_joint_positions(self):
        if self.end_goal:
            self.get_logger().info('Reached Goal!!')
            self.timer.cancel()
        else:
            self.get_logger().info('Running')
            
        

def main(args=None):
    rclpy.init(args=args)
    robot_control = RobotControl()
    rclpy.spin(robot_control)
    robot_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

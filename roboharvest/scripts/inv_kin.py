#!/usr/bin/env python3

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



class RobotControl(Node):
    def __init__(self):
        super().__init__('Robot_controller')

        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,history=HistoryPolicy.KEEP_LAST,depth=10)
        self.timer = self.create_timer(1.0, self.publish_joint_positions)

    def publish_joint_positions(self):
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_position_pub.publish(msg)

        # Define symbols for joint angles T1 to T6
        T1, T2, T3, T4, T5, T6 = symbols('T1 T2 T3 T4 T5 T6')

        # DH parameters
        ai = [0, -612.7, -571.6, 0, 0, 0]
        Ai = [-rad(90), rad(180), rad(180), rad(90), -rad(90), 0]
        di = [128, 0, 0, 163.9, 115.7, 192.2]
        Ti = [T1, T2+rad(90), T3, T4-rad(90), T5, T6]
        
        # Transformation matrices
        def dh_transform(a, alpha, d, theta):
            return Matrix([
                [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
                [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
                [0, sin(alpha), cos(alpha), d],
                [0, 0, 0, 1]
            ])

        hm_0_1 = dh_transform(ai[0], Ai[0], di[0], Ti[0])
        hm_1_2 = dh_transform(ai[1], Ai[1], di[1], Ti[1])
        hm_2_3 = dh_transform(ai[2], Ai[2], di[2], Ti[2])
        hm_3_4 = dh_transform(ai[3], Ai[3], di[3], Ti[3])
        hm_4_5 = dh_transform(ai[4], Ai[4], di[4], Ti[4])
        hm_5_6 = dh_transform(ai[5], Ai[5], di[5], Ti[5])

        # Final transformation matrix
        hm_0_2 = hm_0_1*hm_1_2
        hm_0_3 = hm_0_1*hm_1_2*hm_2_3
        hm_0_4 = hm_0_1*hm_1_2*hm_2_3*hm_3_4
        hm_0_5 = hm_0_1*hm_1_2*hm_2_3*hm_3_4*hm_4_5
        hm_0_6 = hm_0_1*hm_1_2*hm_2_3*hm_3_4*hm_4_5*hm_5_6

        # Jacobian calculations
        R0_0 = Matrix([[0], [0], [1]])
        R0_1 = hm_0_1[:3, :3]
        R0_2 = (hm_0_1 * hm_1_2)[:3, :3]
        R0_3 = (hm_0_1 * hm_1_2 * hm_2_3)[:3, :3]
        R0_4 = (hm_0_1 * hm_1_2 * hm_2_3 * hm_3_4)[:3, :3]
        R0_5 = (hm_0_1 * hm_1_2 * hm_2_3 * hm_3_4 * hm_4_5)[:3, :3]
        R0_6 = (hm_0_1 * hm_1_2 * hm_2_3 * hm_3_4 * hm_4_5 * hm_5_6)[:3, :3]

        d0_0 = Matrix([[0], [0], [1]])
        d0_1 = hm_0_1[:3, -1:]
        d0_2 = hm_0_2[:3, -1:]
        d0_3 = hm_0_3[:3, -1:]
        d0_4 = hm_0_4[:3, -1:]
        d0_5 = hm_0_5[:3, -1:]
        d0_6 = hm_0_6[:3, -1:]


        # Jacobian Matrix
        J1 = R0_0[:3, -1].cross(d0_6 - d0_0)
        J2 = R0_1[:3, -1].cross(d0_6 - d0_1)
        J3 = R0_2[:3, -1].cross(d0_6 - d0_2)
        J4 = R0_3[:3, -1].cross(d0_6 - d0_3)
        J5 = R0_4[:3, -1].cross(d0_6 - d0_4)
        J6 = R0_5[:3, -1].cross(d0_6 - d0_5)
        J1_ = R0_0[:3, -1]
        J2_ = R0_1[:3, -1]
        J3_ = R0_2[:3, -1]
        J4_ = R0_3[:3, -1]
        J5_ = R0_4[:3, -1]
        J6_ = R0_5[:3, -1]

        # Construct the Jacobian matrix
        J_matrix = Matrix([[J1[0,0], J2[0,0], J3[0,0], J4[0,0], J5[0,0], J6[0,0]],
                        [J1[1,0], J2[1,0], J3[1,0], J4[1,0], J5[1,0], J6[1,0]],
                        [J1[2,0], J2[2,0], J3[2,0], J4[2,0], J5[2,0], J6[2,0]],
                            [J1_[0,0], J2_[0,0],J3_[0,0],J4_[0,0],J5_[0,0],J6_[0,0]],
                            [J1_[1,0], J2_[1,0],J3_[1,0],J4_[1,0],J5_[1,0],J6_[1,0]],
                            [J1_[2,0], J2_[2,0],J3_[2,0],J4_[2,0],J5_[2,0],J6_[2,0]]])

        Ti = {T1: 0, T2: 0, T3: 0, T4: 0, T5: 0, T6: 0}
        J_home = J_matrix.subs(Ti)
        print("Jacobian Matrix at Home Position\n")
        pprint(J_home)
        J_value = np.matrix(J_home).astype(np.float64)
        J_inv = np.linalg.pinv(J_value)

        # Initial values

        q = np.zeros((6,1))
        av = (2*np.pi)/20
        r = 100
        Ti = 0
        dt = 0.1
        Time = 20

        # Lists to store trajectory points
        x_values = []
        y_values = []
        z_values = []

        # print("Starting loop")

        while (Ti <= Time):

            x_dot = r * np.cos(av*Ti) * av
            z_dot = -(r * np.sin(av*Ti) * av)

            E = np.matrix([[x_dot], [0], [z_dot], [0], [0], [0]])

            q_dot = J_inv * E
            q = q + q_dot * dt

            # substitute angles in the jacobian matrix
            [A1, A2, A3, A4, A5, A6] = [q[i].item() for i in range(6)]
            J_home = J_matrix.subs({T1: A1, T2: A2, T3: A3, T4: A4, T5: A5, T6: A6})
            J_value = np.matrix(J_home).astype(np.float64)
            J_inv = np.linalg.pinv(J_value)

            # substitute angle in the final transformation matrix to obtain the end effector position
            T_final = hm_0_6.subs({T1:A1, T2:A2, T3:A3, T4:A4, T5:A5, T6:A6})
            x_values.append(T_final[0,3])
            y_values.append(T_final[1,3])
            z_values.append(T_final[2,3])

            print("x_dot : ", x_dot , " z_dot : ", z_dot, "t ", Ti, "time ", Time)

            Ti = Ti + dt

            # Publish the computed joint angles to the robot
            msg.data = [A1, A2, A3, A4, A5, A6]
            self.joint_position_pub.publish(msg)

        # 2D Plot
        plt.title("2D Robot Trajectory")
        plt.xlabel("X")
        plt.ylabel("Z")
        plt.grid(True)
        plt.plot(x_values, z_values)
        plt.show()
        
        # 3D Plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(x_values, y_values, z_values, label='End Effector Trajectory')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Circle Trajectory')
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    Robot_control = RobotControl()
    rclpy.spin(Robot_control)
    Robot_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
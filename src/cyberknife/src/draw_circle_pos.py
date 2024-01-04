#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sympy import symbols, sympify, lambdify
from math import cos, sin, pi
import numpy as np
import pickle
import os
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float64MultiArray

import matplotlib.pyplot as plt

class DrawCircle(Node):
    def __init__(self):
        super().__init__('draw_circle_node')
        self.get_logger().info('Initializing node')

        # Define Symbols
        self.o = symbols('θ')
        self.d = symbols('d')

        self.get_logger().info('Load Jacobian')
        Jacobain_File_Path = os.path.join(get_package_share_directory('cyberknife'), 'config', 'Jacobianp.txt')
        Transformation_Matrix_File_Path = os.path.join(get_package_share_directory('cyberknife'), 'config', 'TransformationMatrix.txt')

        with open(Jacobain_File_Path, 'rb') as file:
            File_out = pickle.load(file)
        Jacobian_eq = sympify(File_out)
        array = [f'{self.o}{i}' for i in range(6)]
        array.append(self.d)
        self.Jacobian = lambdify(array, Jacobian_eq, 'numpy')

        with open(Transformation_Matrix_File_Path, 'rb') as file:
            File_out = pickle.load(file)
        H_eq = sympify(File_out)
        self.H = lambdify(array, H_eq, 'numpy')
        
        # Parameter Definition
        self.radius = 0.25
        self.T = 20
        self.i = 0
        self.time_period = 0.1
        self.d_dist = 0.0

        self.u = [np.full(6, 0.0001)]
        self.velocity_msg = Float64MultiArray()

        self.x = []
        self.y = []
        self.z = []

        self.get_logger().info("Calculating trajectory")
        
        self.publisher_ = self.create_publisher(Float64MultiArray, '/joint_velocities/commands', 10)
        self.calc_timer_ = self.create_timer(self.time_period, self.calc_circle)
        self.draw_timer_ = self.create_timer(self.time_period, self.draw_circle)

    def draw_circle(self):
        if len(self.u) > self.i:
            self.get_logger().info(f'Joint velocities {list(self.u[self.i].astype(float))}, Iteration {self.i + 1}')
            
            self.velocity_msg.data = list(self.u[self.i].astype(float))
            self.publisher_.publish(self.velocity_msg)

            self.i += 1

    def calc_circle(self):
        angle = 2 * pi * self.i * self.time_period / self.T

        dy = -self.radius * 2 * pi * sin(angle + pi/2)/self.T
        dx = 0
        dz = self.radius * 2 * pi * cos(angle + pi/2)/self.T

        velocity = np.array([dx, dy, dz, 0, 0, 0])
        array = np.append(self.u[self.i], self.d_dist)
        Jacobian_current = self.Jacobian(*array)

        joint_velocity = np.linalg.pinv(np.array(Jacobian_current).astype(np.float64)) @ velocity
        self.u.append( self.u[self.i] + (joint_velocity * self.time_period))

        H_array = np.append(self.u[self.i+1], self.d_dist)
        transformation = self.H(*H_array)
        self.x.append(round(transformation[0][3], 3))
        self.y.append(round(transformation[1][3], 3))
        self.z.append(round(transformation[2][3], 3))

        if len(self.x) == 201:
            self.plotGraph()
    
    def plotGraph(self):
        fig = plt.figure()
        axis = fig.add_subplot(projection='3d')
        axis.plot3D(self.x, self.y, self.z)
        axis.set_xlim([-2,2])
        axis.set_ylim([-1,1])
        axis.set_zlim([0,2])
        axis.set_xlabel('X axis')
        axis.set_ylabel('Y axis')
        axis.set_zlabel('Z axis')

        plt.show()


def main(args=None):
    rclpy.init(args=args)
    arm_mover_node = DrawCircle()
    rclpy.spin(arm_mover_node)
    arm_mover_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

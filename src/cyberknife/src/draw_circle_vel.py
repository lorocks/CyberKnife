#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sympy import symbols, sympify, lambdify
from math import cos, sin, pi
import numpy as np
import pickle
import os
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float64MultiArray
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState


class DrawCircle(Node):
    def __init__(self):
        super().__init__('draw_circle_node')
        self.get_logger().info('Initializing node')

        # Define Symbols
        self.o = symbols('θ')
        self.d = symbols('d')

        self.get_logger().info('Load Jacobian')
        Jacobain_File_Path = os.path.join(get_package_share_directory('cyberknife'), 'config', 'Jacobianp.txt')

        array = [f'{self.o}{i}' for i in range(6)]
        array.append(self.d)
        
        with open(Jacobain_File_Path, 'rb') as file:
            File_out = pickle.load(file)
        Jacobian_eq = sympify(File_out)
        self.Jacobian = lambdify(array, Jacobian_eq, 'numpy')

        self.joint_indices = []
        self.actual_joint_names = ['base2', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        
        # Parameter Definition
        self.radius = 0.25
        self.T = 20
        self.t = 0.2
        self.i = 0
        self.time_period = 0.1
        self.stacked_time = 0.0
        self.initial_sim_time = -1

        self.u = np.full(6, 0.0001)
        self.d_dist = 0.0
        self.velocity_msg = Float64MultiArray()

        self.get_logger().info("Calculating trajectory")

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)

        self.publisher_ = self.create_publisher(Float64MultiArray, '/joint_velocities/commands', 10)
        self.sim_clock_ = self.create_subscription(Clock, 'clock', self.update_position, qos_profile)
        self.joint_state_subscription_ = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

    def joint_state_callback(self, msg):
        if not self.joint_indices:
            joint_names = msg.name
            self.joint_indices = [0] * len(self.actual_joint_names)
            for i, jname in enumerate(self.actual_joint_names):
                for j, name in enumerate(joint_names):
                    if jname == name:
                        self.joint_indices[i] = j
        current_joint_positions = np.array(msg.position)
        ordered_positions = np.zeros_like(current_joint_positions)
        for i, index in enumerate(self.joint_indices):
            ordered_positions[i] = current_joint_positions[index]
        self.u = ordered_positions

    def update_position(self, msg):
        sim_time = msg.clock.sec + (msg.clock.nanosec/1000000000)

        if self.initial_sim_time > 0:
            dt = sim_time - self.initial_sim_time

            angle = 2 * pi * dt / self.T
            dx = -self.radius * 2 * pi * sin(angle) / self.T
            dy = 0
            dz = self.radius * 2 * pi * cos(angle) / self.T

            velocity = np.array([dx, dy, dz, 0, 0, 0])
            array = np.append(self.u, self.d_dist)
            Jacobian_current = self.Jacobian(*array)

            joint_velocity = np.linalg.pinv(np.array(Jacobian_current).astype(np.float64)) @ velocity
            self.u = joint_velocity

            self.get_logger().info(f'Joint velocities {list(self.u.astype(float))}, Iteration {self.i + 1}')
                
            self.velocity_msg.data = list(self.u.astype(float))
            self.publisher_.publish(self.velocity_msg)

            self.i += 1
        else:
            self.initial_sim_time = sim_time


def main(args=None):
    rclpy.init(args=args)
    arm_mover_node = DrawCircle()
    rclpy.spin(arm_mover_node)
    arm_mover_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

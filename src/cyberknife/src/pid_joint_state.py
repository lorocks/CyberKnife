#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sympy import symbols, sympify, lambdify, atan2, sqrt
from math import pi, sqrt
import numpy as np
import pickle
import os
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float64MultiArray
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState


class DrawCircle(Node):
    def __init__(self):
        super().__init__('PID_control')
        self.get_logger().info('Initializing node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('Kr', 0.5),
                ('Kp', 0.5),
                ('Ky', 0.5),
                ('K', 0.5),
            ]
        )

        # Define Symbols
        self.o = symbols('θ')
        self.d = symbols('d')

        Kr = self.get_parameter('Kr').get_parameter_value().double_value
        Kp = self.get_parameter('Kp').get_parameter_value().double_value
        Ky = self.get_parameter('Ky').get_parameter_value().double_value
        K = self.get_parameter('K').get_parameter_value().double_value

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
        self.t = 0.2
        self.i = 0
        self.time_period = 0.1
        self.current_sim_time = 0.0
        self.speed = 0.5
        self.d_dist = 0.25

        self.Kp = np.array([K, K, K, Kr, Kp, Ky]) # 0.05

        self.joint_indices = []
        self.actual_joint_names = ['base2', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        self.u = np.full(6, 0.0001).astype(np.float64)
        self.velocity_msg = Float64MultiArray()

        self.get_logger().info("Calculating trajectory")

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        
        self.publisher_ = self.create_publisher(Float64MultiArray, '/joint_velocities/commands', 10)
        self.sim_clock_ = self.create_subscription(Clock, 'clock', self.update_position, qos_profile)
        self.joint_state_subscription_ = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

    def extract_rpy(self, R):
        """
        Extract Roll, Pitch, Yaw from a rotation matrix.
        Assumes the rotation is in the order XYZ.
        """
        roll = atan2(R[2, 1], R[2, 2])
        pitch = atan2(-R[2, 0], sqrt(R[2, 1]**2 + R[2, 2]**2))
        yaw = atan2(R[1, 0], R[0, 0])
        return round(roll, 4), round(pitch, 4), round(yaw, 4)
    
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
        dt = sim_time - self.current_sim_time
        reference = [ 0.0, -2.0, 1.75, 0.0, pi/4, -pi/2 ] # Need to add reference for roll pitch yaw, makign it a 6 elements array

        array = np.append(self.u, self.d_dist)

        transformation = self.H(*array)
        x_current = round(transformation[0][3], 3)
        y_current = round(transformation[1][3], 3)
        z_current = round(transformation[2][3], 3)
        roll, pitch, yaw = self.extract_rpy(transformation[:-1, :-1])

        self.get_logger().info(f'{x_current}, {y_current}, {z_current}, {roll}, {pitch}, {yaw}')

        if self.current_sim_time > 0:
            x_er = reference[0] - x_current
            y_er = reference[1] - y_current
            z_er = reference[2] - z_current
            r_er = reference[3] - roll
            p_er = reference[4] - pitch
            ya_er = reference[5] - yaw

            error_array = np.array([x_er, y_er, z_er, r_er, p_er, ya_er])

            velocity = self.Kp * error_array
            Jacobian_current = self.Jacobian(*array)
            joint_velocity = np.linalg.pinv(np.array(Jacobian_current).astype(np.float64)) @ velocity

            for i, v in enumerate(joint_velocity):
                if v > 3:
                    joint_velocity[i] = 3.0
                if v < -3:
                    joint_velocity[i] = -3.0

            # self.get_logger().info(f'Joint velocities {list(joint_velocity.astype(float))}')
            
            self.velocity_msg.data = list(joint_velocity.astype(float))
            self.publisher_.publish(self.velocity_msg)

        self.current_sim_time = sim_time
        

def main(args=None):
    rclpy.init(args=args)
    arm_mover_node = DrawCircle()
    rclpy.spin(arm_mover_node)
    arm_mover_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

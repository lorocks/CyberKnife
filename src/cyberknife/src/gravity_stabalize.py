#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.joint_state_subscription = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)
        self.joint_velocity_publisher = self.create_publisher(Float64MultiArray, '/joint_velocities/commands', 10)

        self.current_joint_positions = None
        self.initial_joint_positions = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # Set the initial joint angal
        self.joint_indices = []
        self.actual_joint_names = ['base2', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        
        # PID 
        self.Kp = np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5])   # 0.01-0.1
        self.Ki = np.array([1E-8, 1E-8, 1E-8, 1E-8, 1E-8, 1E-8])      # 0.001-0.1
        self.Kd = np.array([1E-8, 1E-8, 1E-8, 1E-8, 1E-8, 1E-8]) * 0.0      #0.01-1
        # self.Kd = np.array([1E-8, 1E-8, 1E-8, 1E-8, 1E-8, 1E-8])      #0.01-1

        # init PID
        self.previous_error = np.zeros(6)
        self.integral_error = np.zeros(6)

        self.timer = self.create_timer(3.0, self.timer_callback)  # every 3s
        self.last_log_time = self.get_clock().now()

    def timer_callback(self):
        now = self.get_clock().now()
        if now - self.last_log_time >= rclpy.duration.Duration(seconds=3):
            self.last_log_time = now
            if self.current_joint_positions is not None:
                joint_positions_in_degrees = np.rad2deg(self.current_joint_positions)
                rounded_joint_positions_in_degrees = [round(v, 2) for v in joint_positions_in_degrees]
                self.get_logger().info(f'Current joint positions in degrees: {rounded_joint_positions_in_degrees}')
            if self.current_joint_velocities is not None:
                rounded_current_joint_velocities = [round(v, 2) for v in self.current_joint_velocities]
                self.get_logger().info(f'Joint rotations: {rounded_current_joint_velocities}')

    def joint_state_callback(self, msg):
        if not self.joint_indices:
            joint_names = msg.name
            self.joint_indices = [0] * len(self.actual_joint_names)
            for i, jname in enumerate(self.actual_joint_names):
                for j, name in enumerate(joint_names):
                    if jname == name:
                        self.joint_indices[i] = j
        self.current_joint_positions = np.array(msg.position)
        ordered_positions = np.zeros_like(self.current_joint_positions)
        for i, index in enumerate(self.joint_indices):
            ordered_positions[i] = self.current_joint_positions[index]
        self.current_joint_positions = ordered_positions
        if self.current_joint_positions is not None:
            self.move_to_initial_position()

    def move_to_initial_position(self):
        error = self.initial_joint_positions - self.current_joint_positions

        # PID
        self.integral_error += error
        derivative_error = error - self.previous_error
        velocity_command = (self.Kp * error) + (self.Ki * self.integral_error) + (self.Kd * derivative_error)

        # :D g
        gravity_compensation = self.calculate_gravity_compensation(self.current_joint_positions)
        velocity_command += gravity_compensation

        self.send_joint_velocities(velocity_command)
        
        # save error
        self.previous_error = error


    def calculate_gravity_compensation(self, current_joint_positions):
        g = -9.81 
        gravity_compensations = np.zeros(6)
        com_positions_masses = [
            # (0.0082451, 0.15, 1.9295E-17, 334.45),  # base_link
            (-0.021051, 0.33235, -0.048325, 663.35),  # base2_link
            (-5.5511E-17, 0.625, 0.27716, 208.9),  # joint1_link
            (0.043481, 0, -0.0038586, 206.09),  # joint2_link
            (0.14843, 7.5699E-07, -6.3398E-08, 9.8998),  # joint3_link
            (0.063752, -2.2204E-16, 4.9549E-18, 5.4704),  # joint4_link
            (0.1006, 0.060159, -7.521E-09, 65.816)  # joint5_link
        ]
        for i, (com_x, com_y, com_z, mass) in enumerate(com_positions_masses):
            com_position = np.array([com_x, com_y, com_z])
            force = mass * g 
            # torque = np.cross(com_position, [0, 0, 0])
            torque = np.cross(com_position, [0, 0, force])
            gravity_compensations[i] = torque[2]

        return gravity_compensations


    def send_joint_velocities(self, velocities):
        msg = Float64MultiArray()
        msg.data = velocities.tolist() if isinstance(velocities, np.ndarray) else list(velocities)
        self.joint_velocity_publisher.publish(msg)
        self.current_joint_velocities = msg.data

def main(args=None):
    rclpy.init(args=args)
    print("ok1")
    robot_controller = RobotController()
    print("ok2")
    rclpy.spin(robot_controller)
    print("ok3")
    robot_controller.destroy_node()
    print("ok4")
    rclpy.shutdown()
    print("ok5")

if __name__ == '__main__':
    main()
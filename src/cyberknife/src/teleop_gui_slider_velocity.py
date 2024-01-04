#!/usr/bin/env python3

import tkinter as tk
from tkinter import Scale
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# Define step size for joint velocity
JOINT_VEL_STEP_SIZE = 0.1

class SliderControlNode(Node):

    def __init__(self):
        super().__init__('slider_control_node')

        # Publisher for the joint velocities
        self.joint_velocity_pub = self.create_publisher(Float64MultiArray, '/joint_velocities/commands', 10)
        self.joint_velocities = [0.0] * 6  # Initial velocities for 6 joints

        # Initialize tkinter
        self.root = tk.Tk()
        self.root.title("Robot Arm Control")

        for i in range(6):
            slider = Scale(self.root, from_=-1, to=1, resolution=JOINT_VEL_STEP_SIZE, 
                           orient=tk.HORIZONTAL, label="Joint " + str(i+1),
                           command=lambda value, i=i: self.update_joint_velocity(i, float(value)))
            slider.pack()

    def update_joint_velocity(self, joint_index, value):
        self.joint_velocities[joint_index] = value
        self.publish_joint_velocities()

    def publish_joint_velocities(self):
        joint_velocity_msg = Float64MultiArray()
        joint_velocity_msg.data = self.joint_velocities
        self.joint_velocity_pub.publish(joint_velocity_msg)
        self.get_logger().info('Joint Velocities: %s' % str(self.joint_velocities))

    def run_slider_control(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = SliderControlNode()
    node.run_slider_control()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
import select
import tty
import termios

# Define step size for joint velocity
JOINT_VEL_STEP_SIZE = 0.1

class KeyboardControlNode(Node):

    def __init__(self):
        super().__init__('keyboard_control_node')

        # Publisher for the joint velocities
        self.joint_velocity_pub = self.create_publisher(Float64MultiArray, '/joint_velocities/commands', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.joint_velocities = [0.0] * 6  # Initial velocities for 6 joints

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
        Control Your Robot Arm!
        ---------------------------
        Joint 1: a/s
        Joint 2: z/x
        Joint 3: d/f
        Joint 4: c/v
        Joint 5: g/h
        Joint 6: b/n
        q : force stop
        Esc to quit
        """

        self.get_logger().info(self.msg)

        while True:
            key = self.getKey()
            if key == '\x1b':  # Escape key
                break
            elif key == 'q':  # Quit
                self.joint_velocities = [0.0] * 6
            else:
                # Update joint velocities based on key pressed
                for i, (increase_key, decrease_key) in enumerate([('a', 's'), ('z', 'x'), ('d', 'f'), ('c', 'v'), ('g', 'h'), ('b', 'n')]):
                    if key == increase_key:
                        self.joint_velocities[i] += JOINT_VEL_STEP_SIZE
                    elif key == decrease_key:
                        self.joint_velocities[i] -= JOINT_VEL_STEP_SIZE

            # Limit joint velocities to a specific range if necessary

            # Publish joint velocities
            joint_velocity_msg = Float64MultiArray()
            joint_velocity_msg.data = self.joint_velocities
            self.joint_velocity_pub.publish(joint_velocity_msg)
            self.get_logger().info('Joint Velocities: %s' % str(self.joint_velocities))

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    node.run_keyboard_control()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

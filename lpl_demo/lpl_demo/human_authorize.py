#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import sys, select, termios, tty

class HumanAuthorize(Node):
    def __init__(self):
        super().__init__('human_authorize')
        self.pub = self.create_publisher(Empty, '/lpl/authorize', 10)
        print("\n\nLPL INTERACTION CONSOLE")
        print("-----------------------")
        print("When the robot pauses, press [SPACE] to authorize it to proceed.")
        print("Press [Ctrl+C] to exit.\n")

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key == ' ':
                        self.pub.publish(Empty())
                        print("\r  >> AUTHORIZATION SENT <<  ", end="", flush=True)
                    elif key == '\x03': # Ctrl+C
                        break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def main(args=None):
    rclpy.init(args=args)
    node = HumanAuthorize()
    node.run()
    node.destroy_node()
    rclpy.shutdown()
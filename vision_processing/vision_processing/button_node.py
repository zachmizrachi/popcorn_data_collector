#!/usr/bin/env python3
import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class ButtonNode(Node):
    def __init__(self):
        super().__init__('button_node')
        self.publisher_ = self.create_publisher(Bool, 'detect_pop_trigger', 10)
        self.get_logger().info("=== Button Node Ready ===")
        self.get_logger().info("Press SPACE to trigger detect_pop()")

        # Setup terminal for non-blocking input
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        # Timer to check for keypresses
        self.timer = self.create_timer(0.1, self.check_keypress)

    def check_keypress(self):
        msg = Bool()
        msg.data = False  # Default to False

        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            if key == ' ':
                msg.data = True
                self.get_logger().info("SPACE pressed → Published True")
            elif key == '\x1b':  # ESC to quit
                self.get_logger().info("ESC pressed → shutting down")
                rclpy.shutdown()
                return

        self.publisher_.publish(msg)



    def destroy_node(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ButtonNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        # --- Publishers ---
        self.state_pub = self.create_publisher(String, '/system_state', 10)
        self.arduino_cmd_pub = self.create_publisher(String, '/arduino_command', 10)

        # --- Subscribers ---
        self.create_subscription(Bool, '/kernel_present', self.kernel_callback, 10)

        # --- Internal state ---
        self.current_state = 'wait_for_kernel'
        self.kernel_present = False

        # --- Timer loop (runs at 2Hz) ---
        self.timer = self.create_timer(0.5, self.state_loop)

        self.get_logger().info("🧭 Controller node started (2-state mode)")

    # ============== Main Loop ==============
    def state_loop(self):
        self.state_pub.publish(String(data=self.current_state))

        if self.current_state == 'wait_for_kernel':
            self.wait_for_kernel()

        elif self.current_state == 'kernel_detected':
            self.kernel_detected()

    # ============== State Handlers ==============

    def wait_for_kernel(self):
        """Run the stepper until a kernel is detected."""
        if not self.kernel_present:
            self.get_logger().info("No kernel yet — running stepper...")
            self.send_arduino_command('1')  # example: run stepper
        else:
            self.get_logger().info("✅ Kernel detected! Stopping stepper.")
            self.send_arduino_command('2')  # example: stop stepper
            self.change_state('kernel_detected')

    def kernel_detected(self):
        """Handle what happens after detection."""
        if not self.kernel_present:
            self.get_logger().info("Kernel lost — resuming search.")
            self.change_state('wait_for_kernel')
        else:
            self.get_logger().debug("Holding state: kernel_detected")

    # ============== Callbacks & Helpers ==============

    def kernel_callback(self, msg: Bool):
        """Update kernel presence flag."""
        self.kernel_present = msg.data

    def send_arduino_command(self, cmd: str):
        """Publish to Arduino interface node."""
        self.arduino_cmd_pub.publish(String(data=cmd))

    def change_state(self, new_state: str):
        if new_state != self.current_state:
            self.get_logger().info(f"🔄 State: {self.current_state} → {new_state}")
            self.current_state = new_state


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

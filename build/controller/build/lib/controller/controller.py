#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        # --- Publishers ---
        self.state_pub = self.create_publisher(String, '/system_state', 10)
        self.arduino_cmd_pub = self.create_publisher(String, '/arduino_command', 10)

        # --- Subscribers ---
        self.create_subscription(String, '/kernel_state', self.kernel_state_callback, 10)

        # --- Internal state ---
        self.current_state = 'initializing'
        self.kernel_state = 'initializing'

        # --- Timer loop (runs at 2Hz) ---
        self.timer = self.create_timer(0.5, self.state_loop)

        # Initialize Arduino servo to receive position
        self.send_arduino_command('3')

        self.get_logger().info("🧭 Controller node started (listening to /kernel_state)")

    # ============== Main Loop ==============
    def state_loop(self):
        self.state_pub.publish(String(data=self.current_state))

        if self.kernel_state == 'initializing':
            self.handle_initializing()

        elif self.kernel_state == 'wait_for_kernel':
            self.wait_for_kernel()

        elif self.kernel_state == 'single_kernel_detected':
            self.single_kernel_detected()

        elif self.kernel_state == 'excess_kernel_detected':
            self.dump_and_reset()

        else:
            self.get_logger().warn(f"Unknown kernel_state: {self.kernel_state}")

    # ============== State Handlers ==============

    def handle_initializing(self):
        """Wait for vision node to stabilize before taking action."""
        self.get_logger().debug("Initializing — waiting for vision to finish setup.")

    def wait_for_kernel(self):
        """Run the stepper until a kernel is detected."""
        self.get_logger().info("No kernel yet — running stepper...")
        self.send_arduino_command('1')  # run stepper motor

    def single_kernel_detected(self):
        """Handle what happens after detection."""
        self.get_logger().info("✅ Single kernel detected! Stopping stepper.")
        self.send_arduino_command('2')  # stop stepper

    def dump_and_reset(self):
        """Handle what happens after detection."""
        self.get_logger().info("Excess kernel detected! Dumping and resetting.")
        self.send_arduino_command('4')  # move to dump
        self.create_timer(3.0, self.dump)
        self.send_arduino_command('6') 
        self.send_arduino_command('3') 

    def dump(self):
        """Stop the motor (triggered by timer)."""
        self.get_logger().info("Moving to dump pos...")
        self.send_arduino_command("5")
        self.get_logger().info("Dumping Kernels...")
        # If you want this timer to only run once, destroy it here:
        for t in self.timers:
            if hasattr(t.callback, "__name__") and t.callback == "dump":
                t.cancel()

    # ============== Callbacks & Helpers ==============

    def kernel_state_callback(self, msg: String):
        """Receives kernel state string from vision node."""
        new_state = msg.data
        if new_state != self.kernel_state:
            self.get_logger().info(f"🧩 Vision state changed: {self.kernel_state} → {new_state}")
        self.kernel_state = new_state

    def send_arduino_command(self, cmd: str):
        """Publish to Arduino interface node."""
        self.arduino_cmd_pub.publish(String(data=cmd))

    def destroy_node(self):
        """Clean up properly."""
        self.get_logger().info("Shutting down controller...")
        super().destroy_node()


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

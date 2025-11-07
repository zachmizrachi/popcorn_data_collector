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
        self.processing_flag = False  # ensures a kernel state is processed only once

        # --- Timer loop (runs at 2Hz) ---
        self.timer = self.create_timer(0.5, self.state_loop)

        # Initialize Arduino servo
        self.send_arduino_command('3')

        self.get_logger().info("🧭 Controller node started (listening to /kernel_state)")

    # ============== Main Loop ==============
    def state_loop(self):

        self.state_pub.publish(String(data=self.current_state))

        if self.current_state == 'idle':
            self.get_logger().info("System in IDLE state.")
            return

        # Process kernel state only if not already processing
        if self.processing_flag:
            return

        # Map kernel_state to handler
        handlers = {
            'initializing': self.handle_initializing,
            'wait_for_kernel': self.wait_for_kernel,
            'single_kernel_detected': self.single_kernel_detected,
            'excess_kernel_detected': self.dump_and_reset,
            'idle': self.idle
        }

        handler = handlers.get(self.kernel_state, self.unknown_kernel_state)
        handler()

    # ============== State Handlers ==============

    def handle_initializing(self):
        self.get_logger().info("Initializing — waiting for vision to stabilize.")

    def wait_for_kernel(self):
        self.get_logger().info("No kernel yet — running stepper...")
        self.send_arduino_command('1')  # run stepper motor

    def single_kernel_detected(self):
        self.processing_flag = True
        self.get_logger().info("✅ Single kernel detected! Stopping stepper.")
        self.send_arduino_command('2')
        self.send_arduino_command('3')
        self.single_kernel_timer = self.create_timer(3.0, self.single_kernel_helper)

    def single_kernel_helper(self):
        self.send_arduino_command('4')  # post-processing
        self.current_state = "idle"
        self.processing_flag = False
        self.get_logger().info("🌙 Returning to idle state.")

        # Cancel and delete the timer
        if hasattr(self, "single_kernel_timer"):
            self.single_kernel_timer.cancel()
            del self.single_kernel_timer

    def dump_and_reset(self):
        self.processing_flag = True
        self.get_logger().info("Excess kernel detected! Dumping and resetting.")
        self.send_arduino_command('5')  # move to dump position
        self.create_timer(3.0, self.dump_complete)

    def dump_complete(self):
        self.send_arduino_command('6')  # reset after dump
        self.send_arduino_command('3')  # go back to initial position
        self.current_state = 'idle'
        self.processing_flag = False
        self.get_logger().info("♻️ Dump complete. Back to idle.")

    def idle(self):
        self.kernel_state = "idle"

    def unknown_kernel_state(self):
        self.get_logger().warn(f"Unknown kernel_state: {self.kernel_state}")

    # ============== Callbacks & Helpers ==============

    def kernel_state_callback(self, msg: String):
        new_state = msg.data
        if new_state != self.kernel_state:
            self.get_logger().info(f"🧩 Vision state changed: {self.kernel_state} → {new_state}")
            self.kernel_state = new_state

    def send_arduino_command(self, cmd: str):
        # try:
        self.arduino_cmd_pub.publish(String(data=cmd))
        #     self.get_logger().info(f"Published Arduino command: {cmd}")
        # except Exception as e:
        #     self.get_logger().error(f"Failed to publish Arduino command: {e}")


    def destroy_node(self):
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

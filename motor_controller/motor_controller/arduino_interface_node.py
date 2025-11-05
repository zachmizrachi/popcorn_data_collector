#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import threading
import sys


class ArduinoInterfaceNode(Node):
    def __init__(self):
        super().__init__('arduino_interface_node')

        # === Serial setup ===
        self.serial_port = '/dev/ttyACM0'  # Adjust if needed
        self.baud_rate = 9600

        try:
            self.arduino = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            time.sleep(2)  # wait for Arduino reset
            self.get_logger().info(f"✅ Connected to Arduino on {self.serial_port}")
        except serial.SerialException:
            self.get_logger().error(f"❌ Failed to connect to Arduino on {self.serial_port}")
            self.arduino = None

        # === ROS Subscriber ===
        self.create_subscription(String, '/arduino_command', self.ros_command_callback, 10)
        self.get_logger().info("📡 Subscribed to /arduino_command topic")

        # === Serial read timer ===
        self.create_timer(0.1, self.read_from_arduino)

        # === Keyboard listener ===
        self.keep_running = True
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.keyboard_thread.start()

        # Show initial menu
        self.print_command_menu()

    # ---------------- Command Menu ----------------
    def print_command_menu(self):
        print("\n=== Command Menu ===")
        print("1 - Run Stepper")
        print("2 - Stop Stepper")
        print("3 - Move Receive Servo → RECEIVE_POS")
        print("4 - Move Receive Servo → BASE_POS")
        print("5 - Move Dump Servo → DUMP_POS")
        print("6 - Move Dump Servo → BASE_POS")
        print("7 - Run DC Motor")
        print("8 - Stop DC Motor")
        print("q - Quit")
        print("====================\n")

    # ---------------- Keyboard Input ----------------
    def keyboard_listener(self):
        """Run in a background thread to capture keyboard input."""
        while self.keep_running:
            try:
                key = input().strip().lower()
                if not key:
                    continue

                if key == 'q':
                    print("👋 Exiting...")
                    self.keep_running = False
                    rclpy.shutdown()
                    break

                if key in [str(i) for i in range(1, 9)]:
                    self.send_command(key)
                else:
                    print("❓ Invalid input. Use 1–8 or q to quit.")
                    self.print_command_menu()

            except Exception as e:
                self.get_logger().error(f"Keyboard input error: {e}")

    # ---------------- ROS Topic Input ----------------
    def ros_command_callback(self, msg: String):
        """Handle commands received from ROS topic."""
        command = msg.data.strip()
        if not command:
            self.get_logger().warn("⚠️ Empty command received on /arduino_command")
            return

        self.get_logger().info(f"📩 Received ROS command: '{command}'")
        self.send_command(command)

    # ---------------- Serial Write ----------------
    def send_command(self, command):
        """Send a single-character command to Arduino."""
        if not self.arduino:
            self.get_logger().error("Arduino not connected.")
            return

        try:
            self.arduino.write(command.encode())
            self.get_logger().info(f"➡ Sent command to Arduino: {command}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")

    # ---------------- Serial Read ----------------
    def read_from_arduino(self):
        """Continuously read responses from Arduino."""
        if not self.arduino:
            return

        try:
            while self.arduino.in_waiting:
                line = self.arduino.readline().decode(errors='ignore').strip()
                if line:
                    print(f"🟢 Arduino: {line}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial read error: {e}")

    # ---------------- Cleanup ----------------
    def destroy_node(self):
        """Clean up serial and threads."""
        self.keep_running = False
        if self.arduino:
            self.arduino.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoInterfaceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

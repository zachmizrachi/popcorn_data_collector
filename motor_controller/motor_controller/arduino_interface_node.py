#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import threading

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

        # State: manual (keyboard) or automated (topic)
        self.manual_mode = True

        # Start keyboard listener
        self.keep_running = True
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.keyboard_thread.start()

        # Subscribe to topic for automated commands
        self.subscription = self.create_subscription(
            String,
            '/arduino_command',
            self.topic_command_callback,
            10
        )

        # Start a timer to continuously read serial data
        self.create_timer(0.1, self.read_from_arduino)

        self.print_command_menu()

    def print_command_menu(self):
        print("raspberry pi: q - Toggle manual/automated mode")
        print("====================\n")
        self.get_logger().info(f"Current mode: {'Manual' if self.manual_mode else 'Automated'}")

    def keyboard_listener(self):
        """Run in a background thread to capture keyboard input."""
        while self.keep_running:
            try:
                key = input().strip().lower()
                if not key:
                    continue

                if key == 'q':
                    self.manual_mode = not self.manual_mode
                    mode = 'Manual' if self.manual_mode else 'Automated'
                    print(f"🔄 Toggled mode: {mode}")
                    continue

                if self.manual_mode:
                    if key in [str(i) for i in range(0, 9)]:
                        self.send_command(key)
                    else:
                        print("❓ Invalid input. Use 0–8 or q to toggle mode.")
                        self.print_command_menu()
                else:
                    print("⚠️  Keyboard disabled in Automated mode. Press 'q' to return to Manual mode.")

            except Exception as e:
                self.get_logger().error(f"Keyboard input error: {e}")

    def topic_command_callback(self, msg: String):
        """Send commands received via ROS topic only if in automated mode."""
        if not self.manual_mode:
            command = msg.data.strip()
            if command in [str(i) for i in range(0, 9)]:
                self.send_command(command)
            else:
                self.get_logger().warn(f"Invalid command from topic: {command}")

    def send_command(self, command):
        """Send a single-character command to Arduino."""
        if not self.arduino:
            self.get_logger().error("Arduino not connected.")
            return
        try:
            self.arduino.write(command.encode())
            self.get_logger().info(f"➡ Sent command: {command}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")

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

    def destroy_node(self):
        """Clean up serial."""
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

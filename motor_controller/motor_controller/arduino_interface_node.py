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
        self.serial_port = '/dev/ttyACM0'
        self.baud_rate = 9600

        try:
            self.arduino = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            time.sleep(2)
            self.get_logger().info(f"✅ Connected to Arduino on {self.serial_port}")
        except serial.SerialException:
            self.get_logger().error(f"❌ Failed to connect to Arduino on {self.serial_port}")
            self.arduino = None

        # === Command set (SINGLE SOURCE OF TRUTH) ===
        self.valid_commands = {
            '0','1','2','3','4','5','6','7','8',
            'C','O','F','V'
        }

        # State: manual (keyboard) or automated (topic)
        self.manual_mode = True

        # Keyboard listener thread
        self.keep_running = True
        self.keyboard_thread = threading.Thread(
            target=self.keyboard_listener,
            daemon=True
        )
        self.keyboard_thread.start()

        # ROS subscription
        self.subscription = self.create_subscription(
            String,
            '/arduino_command',
            self.topic_command_callback,
            10
        )

        # Serial read timer
        self.create_timer(0.1, self.read_from_arduino)

        self.print_command_menu()

    def print_command_menu(self):
        print("raspberry pi:")
        print("  q  - Toggle manual / automated mode")
        print("  Valid commands:", sorted(self.valid_commands))
        print("====================\n")
        self.get_logger().info(
            f"Current mode: {'Manual' if self.manual_mode else 'Automated'}"
        )

    # =========================
    # Keyboard input (thread)
    # =========================
    def keyboard_listener(self):
        while self.keep_running:
            try:
                key = input().strip()
                if not key:
                    continue

                key = key.upper()

                if key == 'Q':
                    self.manual_mode = not self.manual_mode
                    mode = 'Manual' if self.manual_mode else 'Automated'
                    print(f"🔄 Toggled mode: {mode}")
                    continue

                if not self.manual_mode:
                    print("⚠️ Keyboard disabled in Automated mode (press 'q')")
                    continue

                self.process_command(key, source="Keyboard")

            except Exception as e:
                self.get_logger().error(f"Keyboard input error: {e}")

    # =========================
    # ROS topic callback
    # =========================
    def topic_command_callback(self, msg: String):
        if self.manual_mode:
            return

        command = msg.data.strip().upper()
        self.process_command(command, source="ROS Topic")

    # =========================
    # Unified command handler
    # =========================
    def process_command(self, command: str, source: str):
        if command not in self.valid_commands:
            self.get_logger().warn(
                f"❓ Invalid command from {source}: {command}"
            )
            return

        self.send_command(command)

    # =========================
    # Serial write
    # =========================
    def send_command(self, command: str):
        if not self.arduino:
            self.get_logger().error("Arduino not connected.")
            return

        try:
            self.arduino.write(command.encode())
            self.get_logger().info(f"➡ Sent command: {command}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")

    # =========================
    # Serial read
    # =========================
    def read_from_arduino(self):
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

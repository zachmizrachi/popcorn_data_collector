#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Bool, String
from cv_bridge import CvBridge
from your_package.vision_algorithms import run_kernel_count, run_pop_detection


class VisionProcessingNode(Node):
    def __init__(self):
        super().__init__('vision_processing_node')
        self.bridge = CvBridge()

        # --- Subscribers ---
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.create_subscription(String, '/system_state', self.state_callback, 10)

        # --- Publishers ---
        self.kernel_pub = self.create_publisher(Int32, '/camera/kernels_detected', 10)
        self.pop_pub = self.create_publisher(Bool, '/camera/pop_detected', 10)

        # --- Internal state ---
        self.current_state = "IDLE"

        self.get_logger().info("🧠 VisionProcessingNode started")

    # ===================== Callbacks =====================

    def state_callback(self, msg: String):
        """Receives system state from controller node."""
        self.current_state = msg.data
        self.get_logger().info(f"🔄 Switched to state: {self.current_state}")

    def image_callback(self, msg: Image):
        """Processes each incoming frame based on current system state."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        if self.current_state == "RECEIVE":
            count = run_kernel_count(frame, visualize=True, logger=self.get_logger())
            self.kernel_pub.publish(Int32(data=count))

        elif self.current_state == "POPPER":
            popped = run_pop_detection(frame, visualize=True, logger=self.get_logger())
            self.pop_pub.publish(Bool(data=popped))

        elif self.current_state == "DUMP":
            # No processing required
            pass

    # ===================== Cleanup =====================

    def destroy_node(self):
        import cv2
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionProcessingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

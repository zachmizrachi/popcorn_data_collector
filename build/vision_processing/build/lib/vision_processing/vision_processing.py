#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class VisionProcessing(Node):
    def __init__(self):
        super().__init__('vision_processing')

        # CV bridge for converting ROS images to OpenCV
        self.bridge = CvBridge()

        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',  # change if your topic differs
            self.image_callback,
            10
        )

        # Publisher for kernel detection state
        self.state_publisher = self.create_publisher(String, '/kernel_state', 10)

        self.get_logger().info("👁️ VisionProcessing node started")

    def image_callback(self, msg: Image):
        """Process incoming images and determine kernel presence."""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Run kernel detection
            kernel_present = self.detect_kernel(cv_image)

            # Publish the state
            state_msg = String()
            state_msg.data = 'kernel_detected' if kernel_present else 'wait_for_kernel'
            self.state_publisher.publish(state_msg)
            self.get_logger().debug(f"Published kernel state: {state_msg.data}")

        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {e}")

    def detect_kernel(self, frame: np.ndarray) -> bool:
        """
        Simple placeholder for kernel detection.
        Replace this with your actual kernel counting or detection logic.
        """

        # Example: Convert to grayscale and threshold
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

        # Count white pixels
        white_pixels = cv2.countNonZero(thresh)

        # If we detect enough white pixels, assume a kernel is present
        kernel_present = white_pixels > 500  # adjust threshold based on your lighting/setup
        return kernel_present


def main(args=None):
    rclpy.init(args=args)
    node = VisionProcessing()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



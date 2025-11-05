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

        # Store initial reference frame for pixel difference
        self.reference_frame = None

        # Track current state
        self.current_state = 'wait_for_kernel'

        self.get_logger().info("👁️ VisionProcessing node started")

    def image_callback(self, msg: Image):
        """Process incoming images and determine kernel presence."""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Run kernel detection based on state
            if self.current_state == 'wait_for_kernel':
                if self.reference_frame is None:
                    # Store first frame as reference
                    self.reference_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                    self.get_logger().info("📌 Reference frame captured for pixel difference")
                else:
                    # Compute pixel difference from reference
                    diff = self.pixel_difference(self.reference_frame, cv_image)
                    self.get_logger().info(f"Pixel difference: {diff}")
                    if diff > 5000:  # threshold for kernel detection
                        self.current_state = 'kernel_detected'

            elif self.current_state == 'kernel_detected':
                # Do something if needed; for now, just publish state
                pass

            # Publish the state
            state_msg = String()
            state_msg.data = self.current_state
            self.state_publisher.publish(state_msg)
            self.get_logger().debug(f"Published kernel state: {state_msg.data}")

        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {e}")

    def pixel_difference(self, reference_gray: np.ndarray, current_frame: np.ndarray) -> float:
        """Compute pixel difference between reference and current frame."""
        current_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
        # Compute absolute difference
        diff = cv2.absdiff(reference_gray, current_gray)
        # Sum all pixel differences
        diff_sum = np.sum(diff)
        return diff_sum


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

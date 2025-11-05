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

        # CV bridge
        self.bridge = CvBridge()

        # Subscribers
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        # Publishers
        self.state_publisher = self.create_publisher(String, '/kernel_state', 10)
        self.debug_image_pub = self.create_publisher(Image, '/receive_kernel_debug', 10)

        # Parameters for processing
        self.center_circle_radius = 100  # pixels, adjust for your setup
        self.blur_ksize = (5, 5)
        self.brightness_factor = 1.0  # 1.0 = no change, >1 = brighter, <1 = darker

        # Reference frame for pixel difference
        self.ref_frame = None
        self.get_logger().info("👁️ VisionProcessing node started")

    def image_callback(self, msg: Image):
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width = cv_image.shape[:2]

            # Create circular mask
            mask = np.zeros((height, width), dtype=np.uint8)
            cv2.circle(mask, (width // 2, height // 2), self.center_circle_radius, 255, -1)
            masked_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)

            # Apply blur
            blurred = cv2.GaussianBlur(masked_image, self.blur_ksize, 0)

            # Adjust brightness
            bright = cv2.convertScaleAbs(blurred, alpha=self.brightness_factor, beta=0)

            # Set reference frame if None
            if self.ref_frame is None:
                self.ref_frame = cv2.cvtColor(bright, cv2.COLOR_BGR2GRAY)
                self.get_logger().info("Reference frame set for pixel difference")
                state_msg = String()
                state_msg.data = 'wait_for_kernel'
                self.state_publisher.publish(state_msg)
                return

            # Compute pixel difference
            gray = cv2.cvtColor(bright, cv2.COLOR_BGR2GRAY)
            diff = cv2.absdiff(gray, self.ref_frame)
            diff_value = np.sum(diff)
            self.get_logger().info(f"Pixel difference: {diff_value}")

            # Simple threshold to decide kernel presence
            kernel_present = diff_value > 50000  # adjust based on your lighting/setup
            state_msg = String()
            state_msg.data = 'kernel_detected' if kernel_present else 'wait_for_kernel'
            self.state_publisher.publish(state_msg)

            # Publish debug image
            debug_msg = self.bridge.cv2_to_imgmsg(bright, encoding='bgr8')
            self.debug_image_pub.publish(debug_msg)

        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {e}")


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

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

        # Parameters
        self.center_circle_radius = 100   # circle radius in pixels
        self.blur_ksize = (5, 5)          # Gaussian blur kernel size
        self.brightness_factor = 1.2      # Brightness control factor

        # For pixel difference
        self.ref_frame = None
        self.get_logger().info("👁️ VisionProcessing node started with yellow filter enabled")

    def image_callback(self, msg: Image):
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width = cv_image.shape[:2]

            # === Step 1: Create circular mask ===
            mask = np.zeros((height, width), dtype=np.uint8)
            cv2.circle(mask, (width // 2, height // 2), self.center_circle_radius, 255, -1)
            masked_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)

            # === Step 2: Convert to HSV for color filtering ===
            hsv = cv2.cvtColor(masked_image, cv2.COLOR_BGR2HSV)

            # Define yellow color range in HSV
            lower_yellow = np.array([20, 100, 100])  # lower bound (H, S, V)
            upper_yellow = np.array([35, 255, 255])  # upper bound (H, S, V)
            yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

            # Apply yellow mask to image
            yellow_filtered = cv2.bitwise_and(masked_image, masked_image, mask=yellow_mask)

            # === Step 3: Apply blur and brightness ===
            blurred = cv2.GaussianBlur(yellow_filtered, self.blur_ksize, 0)
            bright = cv2.convertScaleAbs(blurred, alpha=self.brightness_factor, beta=0)

            # === Step 4: Set reference frame if none exists ===
            if self.ref_frame is None:
                self.ref_frame = cv2.cvtColor(bright, cv2.COLOR_BGR2GRAY)
                self.get_logger().info("Reference frame set (yellow-filtered masked region)")
                state_msg = String()
                state_msg.data = 'wait_for_kernel'
                self.state_publisher.publish(state_msg)
                return

            # === Step 5: Pixel difference ===
            gray = cv2.cvtColor(bright, cv2.COLOR_BGR2GRAY)
            diff = cv2.absdiff(gray, self.ref_frame)
            diff_value = np.sum(diff)
            self.get_logger().info(f"Pixel difference: {diff_value}")

            # === Step 6: Simple threshold-based kernel presence ===
            kernel_present = diff_value > 50000  # adjust threshold
            state_msg = String()
            state_msg.data = 'kernel_detected' if kernel_present else 'wait_for_kernel'
            self.state_publisher.publish(state_msg)

            # === Step 7: Publish debug view ===
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

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

        # Processing parameters
        self.center_circle_radius = 100
        self.blur_ksize = (5, 5)
        self.brightness_factor = 1.2

        # HSV range for yellow/white detection
        self.lower_yellow = np.array([15, 40, 120])
        self.upper_yellow = np.array([45, 255, 255])

        # Initialization / calibration
        self.ref_frame = None
        self.init_diffs = []
        self.init_frame_count = 30  # number of frames to average during "initializing"
        self.avg_diff = None
        self.state = "initializing"

        self.get_logger().info("👁️ VisionProcessing node started (state: initializing)")

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width = cv_image.shape[:2]

            # === Step 1: Mask ===
            mask = np.zeros((height, width), dtype=np.uint8)
            cv2.circle(mask, (width // 2, height // 2), self.center_circle_radius, 255, -1)
            masked_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)

            # === Step 2: Yellow/white filter ===
            hsv = cv2.cvtColor(masked_image, cv2.COLOR_BGR2HSV)
            yellow_mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
            yellow_filtered = cv2.bitwise_and(masked_image, masked_image, mask=yellow_mask)

            # === Step 3: Blur and brightness ===
            blurred = cv2.GaussianBlur(yellow_filtered, self.blur_ksize, 0)
            bright = cv2.convertScaleAbs(blurred, alpha=self.brightness_factor, beta=0)
            gray = cv2.cvtColor(bright, cv2.COLOR_BGR2GRAY)

            # === Step 4: Reference frame handling ===
            if self.ref_frame is None:
                self.ref_frame = gray.copy()
                self.get_logger().info("Reference frame captured")
                return

            diff = cv2.absdiff(gray, self.ref_frame)
            diff_value = np.sum(diff)

            # === State machine ===
            if self.state == "initializing":
                self.init_diffs.append(diff_value)
                self.get_logger().info(f"Initializing... frame {len(self.init_diffs)}/{self.init_frame_count}")
                if len(self.init_diffs) >= self.init_frame_count:
                    self.avg_diff = np.mean(self.init_diffs)
                    self.state = "wait_for_kernel"
                    self.get_logger().info(f"Initialization complete ✅ avg diff = {self.avg_diff:.2f}")
                    self.publish_state("wait_for_kernel")

            elif self.state == "wait_for_kernel":
                # Calculate percent change relative to average baseline
                if self.avg_diff and self.avg_diff > 0:
                    percent_change = ((diff_value - self.avg_diff) / self.avg_diff) * 100.0
                    self.get_logger().info(f"Percent change: {percent_change:.2f}%")
                else:
                    percent_change = 0.0

                # Decide kernel presence based on % increase
                kernel_present = percent_change > 100.0  # adjust sensitivity threshold
                if kernel_present:
                    self.state = "kernel_detected"
                    self.publish_state("kernel_detected")
                    self.get_logger().info("🌽 Kernel detected!")
                else:
                    self.publish_state("wait_for_kernel")

            elif self.state == "kernel_detected":
                # You can choose to reset after detection if needed
                pass

            # === Step 5: Publish debug image ===
            debug_msg = self.bridge.cv2_to_imgmsg(bright, encoding='bgr8')
            self.debug_image_pub.publish(debug_msg)

        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {e}")

    def publish_state(self, state_str):
        msg = String()
        msg.data = state_str
        self.state_publisher.publish(msg)


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

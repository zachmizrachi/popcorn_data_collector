#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from scipy.signal import find_peaks
from scipy.ndimage import gaussian_filter1d
from std_msgs.msg import Bool

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

        self.controller_callback_sub = self.create_subscription(String, '/vision_node_update_signal', self.controller_callback, 10)

        # Publishers
        self.state_publisher = self.create_publisher(String, '/kernel_state', 10)
        self.debug_image_pub = self.create_publisher(Image, '/receive_kernel_debug', 10)

        # Processing parameters
        self.center_circle_radius = 50
        self.blur_ksize = 9
        self.brightness_factor = 1
        self.projection_step_deg = 10
        self.max_percent_change = 20000.0
        self.peak_height = 100  # Adjust this for sensitivity
        self.peak_distance = 10  # Min distance between peaks

        # HSV range for yellow/white detection
        self.lower_yellow = np.array([15, 50, 150])
        self.upper_yellow = np.array([45, 255, 255])

        # Initialization / calibration
        self.ref_frame = None
        self.init_diffs = []
        self.init_frame_count = 10
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
            blurred = cv2.medianBlur(yellow_filtered, self.blur_ksize)
            bright = cv2.convertScaleAbs(blurred, alpha=self.brightness_factor, beta=0)
            gray = cv2.cvtColor(bright, cv2.COLOR_BGR2GRAY)

            # === Step 4: Reference frame handling ===
            if self.ref_frame is None:
                self.ref_frame = gray.copy()
                self.get_logger().info("Reference frame captured")
                return

            diff = cv2.absdiff(gray, self.ref_frame)
            diff_value = np.sum(diff)
            self.get_logger().info(f"vision incoming state: {self.state}")

            # === State machine ===
            if self.state == "initializing":
                self.init_diffs.append(diff_value)
                self.get_logger().info(f"Initializing... frame {len(self.init_diffs)}/{self.init_frame_count}: {diff_value}")
                if len(self.init_diffs) >= self.init_frame_count:
                    self.avg_diff = np.mean(self.init_diffs)
                    self.state = "wait_for_kernel"
                    self.publish_state("wait_for_kernel")
                    self.get_logger().info(f"Initialization complete ✅ avg diff = {self.avg_diff}")
                return

            elif self.state == "wait_for_kernel":
                # --- Step 4a: Fast pixel difference check ---
                # self.get_logger().info(f"MADE IT HERE")
                epsilon = 1e-6  # small number to avoid division by zero
                percent_change = ((diff_value - self.avg_diff) / (self.avg_diff + epsilon)) * 100.0
                # self.get_logger().info(f"Pixel difference percent change: {percent_change:.2f}%")

                # If percent change is very small, assume no kernel present
                if percent_change < 1000:  # <-- tweak this threshold as needed
                    self.state = "wait_for_kernel"
                    self.publish_state(self.state)
                    self.debug_frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)  # just publish the gray image for debug
                    return
                else : 
                    self.state = "change_detected"
                    self.publish_state(self.state)

                # If percent change exceeds reasonable range, immediately mark as excess
                # elif percent_change > self.max_percent_change:
                #     self.state = "excess_kernel_detected"
                #     self.publish_state(self.state)
                #     self.get_logger().info("⚠️ EXCESS Kernel detected due to large percent change")
                #     return

                # --- Step 4b: If diff indicates kernel present, run heavier projection detection ---
                    
            elif self.state == "count_kernels":
                self.state = self.detect_kernels_by_projection(gray)
                self.publish_state(self.state)

            # === Step 5: Publish debug image ===
            debug_msg = self.bridge.cv2_to_imgmsg(self.debug_frame, encoding='bgr8')
            self.debug_image_pub.publish(debug_msg)

        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {e}")

    def detect_kernels_by_projection(self, fg_image):
        """
        Project the foreground image along multiple lines passing through the center.
        Use image rotation + column sum for fast projection.
        """
        # Convert fg_image to BGR for debug visualization
        self.debug_frame = cv2.cvtColor(fg_image, cv2.COLOR_GRAY2BGR)

        # Create circular mask
        height, width = fg_image.shape
        mask = np.zeros((height, width), dtype=np.uint8)
        cv2.circle(mask, (width // 2, height // 2), self.center_circle_radius, 255, -1)

        # Apply mask to debug frame to make lines appear behind
        masked_debug_frame = cv2.bitwise_and(self.debug_frame, self.debug_frame, mask=mask)

        # Determine mask center
        moments = cv2.moments(fg_image)
        if moments["m00"] != 0:
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
        else:
            cy, cx = fg_image.shape[0] // 2, fg_image.shape[1] // 2

        angles = np.arange(0, 180, self.projection_step_deg)
        peak_summary = {}

        # Create a blank layer for lines
        line_layer = np.zeros_like(self.debug_frame)

        for theta in angles:
            # Rotate image around center
            M = cv2.getRotationMatrix2D((cx, cy), theta, 1.0)
            rotated = cv2.warpAffine(fg_image, M, (fg_image.shape[1], fg_image.shape[0]), flags=cv2.INTER_LINEAR)

            # Project along vertical axis (sum columns)
            projection = rotated.sum(axis=0)
            smoothed = gaussian_filter1d(projection, sigma=2)
            peaks, _ = find_peaks(smoothed, height=self.peak_height, distance=self.peak_distance)
            n_peaks = len(peaks)

            # Update peak summary
            if n_peaks not in peak_summary:
                peak_summary[n_peaks] = 0
            peak_summary[n_peaks] += 1

            # Draw the projection line on line_layer
            theta_rad = np.deg2rad(theta)
            dx = np.cos(theta_rad)
            dy = np.sin(theta_rad)
            x1 = int(cx - dx * 100)
            y1 = int(cy - dy * 100)
            x2 = int(cx + dx * 100)
            y2 = int(cy + dy * 100)
            color = (0, 0, 255) if n_peaks > 1 else (0, 255, 0)
            cv2.line(line_layer, (x1, y1), (x2, y2), color, 1)
            cv2.putText(line_layer, str(n_peaks), (x2, y2), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

        # Combine line_layer with masked debug frame so lines are behind the mask
        self.debug_frame = cv2.addWeighted(line_layer, 0.6, masked_debug_frame, 1.0, 0)

        # Overlay peak summary
        y_offset = 20
        for peaks, count in peak_summary.items():
            cv2.putText(self.debug_frame, f"{count} proj with {peaks} peak(s)", (10, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            y_offset += 15

        if any(k > 1 for k in peak_summary.keys()):
            self.get_logger().info("🔴 Excess kernel detected")
            return "excess_kernel_detected"

        elif all(k == 0 for k in peak_summary.keys()):
            self.get_logger().info("🟡 Waiting for kernel")
            return "wait_for_kernel"

        else:
            self.get_logger().info("🟢 Single kernel detected")
            return "single_kernel_detected"

    # need to change controller code to update controller_callback to change kernel state from change_detected to count_kernels
    def controller_callback(self, msg: String):
        # Detect rising edge
        if msg.data == "reset":
            self.get_logger().info("🔄 Reset received! Performing reset...")
            self.state = "wait_for_kernel"
        elif msg.data == "count_kernels": 
            self.get_logger().info("vision node recieved switch to count_kernels from controller...")
            self.state = "count_kernels"

        # Update previous state
        self.last_reset = msg.data


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

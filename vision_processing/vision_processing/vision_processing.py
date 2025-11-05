from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class VisionProcessing(Node):
    def __init__(self):
        super().__init__('vision_processing')

        self.bridge = CvBridge()

        # Subscribe to raw camera input
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        # Publisher for kernel detection state
        self.state_publisher = self.create_publisher(String, '/kernel_state', 10)

        # Publisher for debug image
        self.debug_image_publisher = self.create_publisher(Image, '/receive_kernel_debug', 10)

        self.get_logger().info("👁️ VisionProcessing node started")

        # Parameters for masking / processing
        self.circle_radius = 100  # pixels, adjustable
        self.blur_kernel = (7, 7)
        self.brightness_factor = 1.2

    def image_callback(self, msg: Image):
        """Process incoming images with circular mask, blur, brightness, and publish debug image."""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # --- Circular mask ---
            height, width = cv_image.shape[:2]
            center_x, center_y = width // 2, height // 2

            mask = np.zeros((height, width), dtype=np.uint8)
            cv2.circle(mask, (center_x, center_y), self.circle_radius, 255, -1)
            masked_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)

            # --- Blurring / smoothing ---
            blurred_image = cv2.GaussianBlur(masked_image, self.blur_kernel, 0)

            # --- Brightness adjustment ---
            processed_image = cv2.convertScaleAbs(blurred_image, alpha=self.brightness_factor, beta=0)

            # --- Publish debug image ---
            debug_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')
            self.debug_image_publisher.publish(debug_msg)

            # --- Optional: continue with kernel detection ---
            kernel_present = self.detect_kernel(processed_image)
            state_msg = String()
            state_msg.data = 'kernel_detected' if kernel_present else 'wait_for_kernel'
            self.state_publisher.publish(state_msg)
            self.get_logger().debug(f"Published kernel state: {state_msg.data}")

        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {e}")

    def detect_kernel(self, frame: np.ndarray) -> bool:
        """Simple placeholder for kernel detection; replace with your actual logic."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        white_pixels = cv2.countNonZero(thresh)
        kernel_present = white_pixels > 500
        return kernel_present

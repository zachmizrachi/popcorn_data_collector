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
import time
from collections import deque


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

        self.debug_frame = None
        self.original_frame_gray = None

        # ---- Rolling buffer parameters ----
        self.lag_frames = 5
        self.frame_buffer = deque(maxlen=self.lag_frames + 1)

        self.controller_callback_sub = self.create_subscription(String, '/vision_node_update_signal', self.controller_callback, 10)
        # self.detect_pop_callback_sub = self.create_subscription(Bool, '/detect_pop_trigger', self.detect_pop_callback, 10)

        self.popped = False

        # Publishers
        self.kernel_state_publisher = self.create_publisher(String, '/kernel_state', 10)
        self.vibration_state_publisher = self.create_publisher(String, '/vibrate_state', 10)
        self.debug_image_pub = self.create_publisher(Image, '/receive_kernel_debug', 10)

        # Processing parameters
        self.center_circle_radius = 50
        self.blur_ksize = 9
        self.brightness_factor = 1
        self.projection_step_deg = 10
        self.max_percent_change = 20000.0
        self.peak_height = 100  # Adjust this for sensitivity
        self.peak_distance = 20  # Min distance between peaks
        self.white_thresh = 100

        self.curr_pop_time = 0

        # self.diff_threshold = 10
        # BLUR_KSIZE = 3
        # LOW_DIFF_THRESHOLD  = 8    # candidate change
        # HIGH_DIFF_THRESHOLD = 25   # must exist inside blob
        # MIN_BLOB_AREA = 100
        # USE_HIGH_DIFF_GATE = True  # set False to ignore high-diff threshold



        # HSV range for yellow/white detection
        self.lower_yellow = np.array([10, 30, 80])
        self.upper_yellow = np.array([50, 255, 255])

        # Initialization / calibration
        self.ref_frame = None
        self.init_diffs = []
        self.init_frame_count = 10
        self.avg_diff = None
        self.state = "initializing"

        self.vibration_state = "idle"   # idle | vibrating

        self.pop_threshold = 35
        self.not_moving_threshold = 5.0  # percent
        self.not_moving_count = 0
        self.not_moving_num_frames = 10
        self.vibrate_start_time = 0
        self.vibrate_block_dur = 3
        
        self.publish_vibration_state("idle")
        self.get_logger().info("👁️ VisionProcessing node started (state: initializing)")

    def create_center_mask(self, shape, radius):
        h, w = shape
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.circle(mask, (w // 2, h // 2), radius, 255, -1)
        return mask
    
    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width = cv_image.shape[:2]

           # === Step 1: Mask ===
            mask = np.zeros((height, width), dtype=np.uint8)
            cv2.circle(mask, (width // 2, height // 2), self.center_circle_radius, 255, -1)
            masked_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)
            masked_gray = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)

            # === Step 2: Yellow/white filter ===
            hsv = cv2.cvtColor(masked_image, cv2.COLOR_BGR2HSV)
            yellow_mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)

            # --- Filter out small noise blobs ---
            # num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(yellow_mask, connectivity=8)
            # min_area = 100  # adjust as needed
            # filtered_mask = np.zeros_like(yellow_mask)
            # for i in range(1, num_labels):  # skip background
            #     if stats[i, cv2.CC_STAT_AREA] >= min_area:
            #         filtered_mask[labels == i] = 255

            yellow_filtered = cv2.bitwise_and(masked_image, masked_image, mask=yellow_mask)

            # === Step 3: Blur, normalize, and brightness ===
            blurred = cv2.medianBlur(yellow_filtered, self.blur_ksize)
            bright = cv2.convertScaleAbs(blurred, alpha=self.brightness_factor, beta=0)
            gray = cv2.cvtColor(bright, cv2.COLOR_BGR2GRAY)

            # --- Lighting compensation ---
            # gray = cv2.equalizeHist(gray)
            gray = cv2.GaussianBlur(gray, (5, 5), 0)

            # === Step 4: Reference frame handling ===
            if self.ref_frame is None:
                self.ref_frame = gray.copy()
                self.get_logger().info("Reference frame captured")
                return

            diff = cv2.absdiff(gray, self.ref_frame)
            diff_value = np.sum(diff)
            # self.get_logger().info(f"diff value: {diff_value}")

            # return
            # === State machine ===
            if self.state == "initializing":
                self.init_diffs.append(diff_value)
                self.get_logger().info(f"Initializing... frame {len(self.init_diffs)}/{self.init_frame_count}: {diff_value}")
                if len(self.init_diffs) >= self.init_frame_count:
                    self.avg_diff = np.mean(self.init_diffs)
                    self.state = "wait_for_kernel"
                    self.publish_kernel_state("wait_for_kernel")
                    self.get_logger().info(f"Initialization complete ✅ avg diff = {self.avg_diff}")

                self.debug_frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                debug_msg = self.bridge.cv2_to_imgmsg(self.debug_frame, encoding='bgr8')
                self.debug_image_pub.publish(debug_msg)

                return

            elif self.state == "wait_for_kernel":
                
                # --- Step 4a: Fast pixel difference check ---
                # self.get_logger().info(f"MADE IT HERE")
                epsilon = 1e-6  # small number to avoid division by zero
                percent_change = ((diff_value - self.avg_diff) / (self.avg_diff + epsilon)) * 100.0
                self.get_logger().info(f"Pixel difference percent change: {percent_change:.2f}%")

                # Initialize timer on first entry into this state
                if not hasattr(self, "wait_for_kernel_start_time") or self.wait_for_kernel_start_time is None:
                    self.wait_for_kernel_start_time = time.time()

                # Compute elapsed time
                elapsed = time.time() - self.wait_for_kernel_start_time

                WAIT_DURATION = 0.5  # seconds (tweak as needed)
                if elapsed < WAIT_DURATION:
                    return
                else : 
                    self.wait_for_kernel_start_time = None  # reset timer for next cycle
                
                # If percent change is very small, assume no kernel present
                if percent_change < 100:  # <-- tweak this threshold as needed
                    self.state = "wait_for_kernel"
                    self.publish_kernel_state(self.state)
                    self.debug_frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)  # just publish the gray image for debug
                    return
                else : 
                    self.state = "change_detected"
                    self.publish_kernel_state(self.state)

                # If percent change exceeds reasonable range, immediately mark as excess
                # elif percent_change > self.max_percent_change:
                #     self.state = "excess_kernel_detected"
                #     self.publish_kernel_state(self.state)
                #     self.get_logger().info("⚠️ EXCESS Kernel detected due to large percent change")
                #     return

                # --- Step 4b: If diff indicates kernel present, run heavier projection detection ---
                    
            elif self.state == "count_kernels":
                self.state = self.detect_kernels_by_projection(gray)
                self.publish_kernel_state(self.state)

            elif self.state == "detect_pop":

                self.frame_buffer.append(gray.copy())

                if len(self.frame_buffer) < self.lag_frames + 1:
                    # self.publish_debug(gray)
                    return

                ref_frame = self.frame_buffer[0]
                cur_frame = self.frame_buffer[-1]

                mask = self.create_center_mask(gray.shape, self.center_circle_radius)

                if self.curr_pop_time == 0 : 
                    self.curr_pop_time = time.time()

                self.state, diff_vis, percent = self.detect_pop(
                    ref_frame,
                    cur_frame,
                    mask,
                    low_diff_threshold=35,
                    high_diff_threshold=25,
                    blur_ksize=3,
                    min_blob_area=150,
                    use_high_diff_gate=False
                )
                self.debug_frame = diff_vis
                self.publish_kernel_state(self.state)
                # return
            
            elif self.state == "pop_done": 
                
                self.curr_pop_time = 0
                self.popped = False
                self.frame_buffer.clear()
                self.not_moving_count = 0
                self.get_logger().info(f"POP DONE !!!!")
                # return

            # === Step 5: Publish debug image ===
            if self.debug_frame is not None: 
                debug_msg = self.bridge.cv2_to_imgmsg(self.debug_frame, encoding='bgr8')
                self.debug_image_pub.publish(debug_msg)
            else : 
                self.debug_image_pub.publish(msg)
            
            return 

        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {e}")

    def detect_kernels_by_projection(self, fg_image):
        """
        Project the foreground image along multiple lines passing through the center.
        Uses image rotation + column sum for fast projection.
        Annotates the debug frame with projection info, peak count, and distances.
        """
        import numpy as np
        import cv2
        from scipy.signal import find_peaks
        from scipy.ndimage import gaussian_filter1d

        # === Prepare debug visualization ===
        self.debug_frame = cv2.cvtColor(fg_image, cv2.COLOR_GRAY2BGR)

        # Create circular mask
        height, width = fg_image.shape
        mask = np.zeros((height, width), dtype=np.uint8)
        cv2.circle(mask, (width // 2, height // 2), self.center_circle_radius, 255, -1)
        masked_debug_frame = cv2.bitwise_and(self.debug_frame, self.debug_frame, mask=mask)

        # Determine center of mass (fallback to image center)
        moments = cv2.moments(fg_image)
        if moments["m00"] != 0:
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
        else:
            cy, cx = fg_image.shape[0] // 2, fg_image.shape[1] // 2

        # === Projection parameters ===
        angles = np.arange(0, 180, self.projection_step_deg)
        peak_summary = {}
        all_peak_distances = []

        # Create line layer for visualization
        line_layer = np.zeros_like(self.debug_frame)
        all_peak_heights = []

        for theta in angles:
            # Rotate around center
            M = cv2.getRotationMatrix2D((cx, cy), theta, 1.0)
            rotated = cv2.warpAffine(fg_image, M, (fg_image.shape[1], fg_image.shape[0]), flags=cv2.INTER_LINEAR)

            # Project along vertical axis
            projection = rotated.sum(axis=0)
            smoothed = gaussian_filter1d(projection, sigma=2)
            peaks, properties = find_peaks(smoothed, height=self.peak_height, distance=self.peak_distance)
            n_peaks = len(peaks)
            peak_heights = properties['peak_heights']  # This gives an array of the heights of the detected peaks
            all_peak_heights.extend(peak_heights)


            # --- Compute and store distances between peaks ---
            if len(peaks) > 1:
                peak_distances = np.diff(peaks) * self.projection_step_deg
                avg_dist = np.mean(peak_distances)
                all_peak_distances.extend(peak_distances)
            else:
                peak_distances = []
                avg_dist = None

            # Update summary
            if n_peaks not in peak_summary:
                peak_summary[n_peaks] = 0
            peak_summary[n_peaks] += 1

            # --- Draw projection direction and label ---
            theta_rad = np.deg2rad(theta)
            dx = np.cos(theta_rad)
            dy = np.sin(theta_rad)
            x1 = int(cx - dx * 100)
            y1 = int(cy - dy * 100)
            x2 = int(cx + dx * 100)
            y2 = int(cy + dy * 100)

            # Color by number of peaks
            if n_peaks > 1:
                color = (0, 0, 255)  # red → multiple kernels
            elif n_peaks == 1:
                color = (0, 255, 0)  # green → single kernel
            else:
                color = (255, 255, 0)  # cyan → none

            cv2.line(line_layer, (x1, y1), (x2, y2), color, 1)
            label = f"{n_peaks}"
            if avg_dist is not None:
                label += f" ({avg_dist:.1f})"
            cv2.putText(line_layer, label, (x2, y2), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

        # === Combine layers ===
        self.debug_frame = cv2.addWeighted(line_layer, 0.6, masked_debug_frame, 1.0, 0)

        # === Overlay peak summary ===
        y_offset = 20
        for peaks, count in sorted(peak_summary.items()):
            cv2.putText(self.debug_frame, f"{count} proj -- {peaks} peak(s)",
                        (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            y_offset += 15

        # If any distances computed, show their mean
        if all_peak_distances:
            mean_dist = np.mean(all_peak_distances)
            mean_height = np.mean(all_peak_heights) if all_peak_heights else 0
            cv2.putText(self.debug_frame, f"Avg peak dist: {mean_dist:.2f}, Avg peak height: {mean_height:.0f}",
                        (10, y_offset + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        else:
            mean_height = np.mean(all_peak_heights) if all_peak_heights else 0
            cv2.putText(self.debug_frame, f"No valid peak distances, Avg peak height: {mean_height:.0f}",
                        (10, y_offset + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        

        # === Determine kernel state ===
        if any(k > 1 for k in peak_summary.keys()):
            self.get_logger().info("🔴 Excess kernel detected")
            return "excess_kernel_detected"

        elif all(k == 0 for k in peak_summary.keys()):
            self.get_logger().info("🟡 Waiting for kernel")
            return "wait_for_kernel"

        else:
            self.get_logger().info("🟢 Single kernel detected")
            return "single_kernel_detected"

    def detect_pop(
        self,
        img_orig,
        img,
        mask,
        low_diff_threshold=8,
        high_diff_threshold=25,
        blur_ksize=3,
        min_blob_area=100,
        use_high_diff_gate=True
    ):

        # ---- VIBRATION STATE MACHINE ----

        now = time.time()

        # Trigger vibration
        if (
            self.vibration_state == "idle" and
            self.not_moving_count > self.not_moving_num_frames
        ):
            self.vibration_state = "vibrating"
            self.vibrate_start_time = now
            self.publish_vibration_state("vibrate_kernel")

        # End vibration
        elif (
            self.vibration_state == "vibrating" and
            now - self.vibrate_start_time > self.vibrate_block_dur
        ):
            self.vibration_state = "idle"
            self.vibrate_start_time = 0
            self.publish_vibration_state("idle")




        
        diff = cv2.absdiff(img, img_orig)

        if blur_ksize > 1:
            diff = cv2.GaussianBlur(diff, (blur_ksize, blur_ksize), 0)

        low_mask = (diff > low_diff_threshold).astype(np.uint8) * 255
        low_mask = cv2.bitwise_and(low_mask, mask)

        if use_high_diff_gate:
            high_mask = (diff > high_diff_threshold).astype(np.uint8) * 255
            high_mask = cv2.bitwise_and(high_mask, mask)
        else:
            high_mask = None

        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(low_mask, 8)
        clean_mask = np.zeros_like(low_mask)

        for label in range(1, num_labels):
            if stats[label, cv2.CC_STAT_AREA] < min_blob_area:
                continue
            component = labels == label
            if use_high_diff_gate and not np.any(high_mask[component]):
                continue
            clean_mask[component] = 255

        percent_changed = (
            np.count_nonzero(clean_mask) /
            max(np.count_nonzero(mask), 1)
        ) * 100.0

        if percent_changed < self.not_moving_threshold:
            self.not_moving_count += 1
        else:
            self.not_moving_count = 0

        # only send vibrate command once, start timer
        # if self.not_moving_count > self.not_moving_num_frames and self.vibrate_start_time == 0: 
        #     self.vibrate_start_time = time.time()
        #     self.publish_vibration_state("vibrate_kernel")

        # only trigger pop if not running vibrate command
        # if percent_changed > self.pop_threshold and self.vibrate_start_time == 0:
        #     self.popped = True

        # if time.time() - self.vibrate_start_time > self.vibrate_block_dur :
        #     self.vibrate_start_time = 0
        #     self.publish_vibration_state("idle")

        if (
            percent_changed > self.pop_threshold and
            self.vibration_state == "idle"
        ):
            self.popped = True

        self.get_logger().info(f"Vibration state: {self.vibration_state}")

        vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        vis[mask == 0] = 0
        vis[clean_mask > 0] = (0, 0, 255)
        time_for_pop = int(time.time() - self.curr_pop_time)

        if self.vibrate_start_time == 0: 
            info_str = (
                f"{percent_changed:3.0f}%, "
                f"{time_for_pop:2d}s, "
                f"still: {self.not_moving_count}"
            )
        else :
            info_str = (
                f"{percent_changed:3.0f}%, "
                f"{time_for_pop:2d}s, "
                f"still: {self.not_moving_count}"
                f"v: {time.time() - self.vibrate_start_time}"
            )

        # info_str = f"{percent_changed:3.0f}%, {time_for_pop:2d} seconds"

        if self.popped:
            cv2.putText(vis, "POPPED! " + info_str, (10, vis.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            return "pop_done", vis, percent_changed
        else:
            cv2.putText(vis, "Waiting for pop: " + info_str, (10, vis.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
            return "detect_pop", vis, percent_changed


    # def detect_pop(self, img_orig, img, mask,
    #            low_diff_threshold=70,
    #            high_diff_threshold=25,
    #            blur_ksize=3,
    #            min_blob_area=100,
    #            use_high_diff_gate=False):
        

    def controller_callback(self, msg: String):
        # Detect rising edge
        if msg.data == "reset":
            self.get_logger().info("🔄 Reset received! Performing reset...")
            self.state = "wait_for_kernel"
        elif msg.data == "count_kernels": 
            self.get_logger().info("vision node recieved switch to count_kernels from controller...")
            self.state = "count_kernels"
        elif msg.data == "detect_pop": 
            self.get_logger().info("vision node recieved switch to detect_pop from controller...")
            self.state = "detect_pop"

        # Update previous state
        self.last_reset = msg.data

    def publish_kernel_state(self, state_str):
        msg = String()
        msg.data = state_str
        self.kernel_state_publisher.publish(msg)

    def publish_vibration_state(self, state_str):
        msg = String()
        msg.data = state_str
        self.vibration_state_publisher.publish(msg)


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

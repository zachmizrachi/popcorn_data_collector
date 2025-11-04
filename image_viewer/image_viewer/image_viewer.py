#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


class ImageViewer(Node):
    def __init__(self):
        super().__init__('image_viewer')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',         # <-- change if your topic name differs
            self.image_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info('ImageViewer node started, listening to /image_raw')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Show image
            cv2.imshow("Camera View", cv_image)
            key = cv2.waitKey(1)
            if key == ord('q'):  # press 'q' to quit
                self.get_logger().info('Quitting image viewer...')
                rclpy.shutdown()

        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge Error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

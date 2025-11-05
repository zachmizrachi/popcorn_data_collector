#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sys


class ImageViewer(Node):
    def __init__(self, topic_name='/image_raw'):
        super().__init__('image_viewer')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.image_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info(f'ImageViewer node started, listening to {topic_name}')

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

    # List of topics to choose from
    available_topics = ['/image_raw', '/receive_kernel_debug', '/camera/color/image_raw']
    print("Select image topic to view:")
    for i, t in enumerate(available_topics):
        print(f"{i + 1}: {t}")
    choice = input(f"Enter number (default 1): ").strip()

    try:
        topic_index = int(choice) - 1
        if 0 <= topic_index < len(available_topics):
            topic_name = available_topics[topic_index]
        else:
            topic_name = available_topics[0]
    except ValueError:
        topic_name = available_topics[0]

    node = ImageViewer(topic_name=topic_name)

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

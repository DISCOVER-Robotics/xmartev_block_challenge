import cv2
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

class VideoSubscriber(Node):
    def __init__(self, compressed_topic_name):
        super().__init__('video_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            compressed_topic_name,
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if image is not None:
            cv2.imshow("Compressed Video", image)
            cv2.waitKey(1)

def main(args):
    rclpy.init()
    node = VideoSubscriber(args.compressed_topic_name)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Run this script to show compressed image topic')
    parser.add_argument('--compressed_topic_name', type=str, required=True, help='Name of the image topic to subscribe to')
    args = parser.parse_args()

    main(args)

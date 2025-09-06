import cv2
import argparse
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage

class ImageCompressor(Node):
    def __init__(self, topic_name):
        super().__init__('image_compressor')
        
        self.subscription = self.create_subscription(
            Image, 
            topic_name, 
            self.image_callback, 
            10
        )
        
        compress_img_topic = topic_name.replace('image_raw', 'image_compressed')
        self.publisher = self.create_publisher(
            CompressedImage, 
            compress_img_topic,
            10
        )
        
        self.bridge = CvBridge()
        self.get_logger().info(f'Image Compressor <{topic_name}> Node Started')

    def image_callback(self, msg):
        try:
            # convert ROS 2 image topic to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            encode_param = [cv2.IMWRITE_JPEG_QUALITY, 50]  # 50% quantity
            ret, encoded_image = cv2.imencode('.jpg', cv_image, encode_param)
            
            if ret:
                compressed_msg = CompressedImage()
                compressed_msg.header = msg.header
                compressed_msg.format = "jpeg"
                compressed_msg.data = encoded_image.tobytes()
                
                self.publisher.publish(compressed_msg)
        
        except Exception as e:
            self.get_logger().error(f"Error compressing image: {e}")

def main(args):
    rclpy.init()
    node = ImageCompressor(args.topic_name)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'Shutting down Image Compressor <{args.topic_name}>...')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Run this script to compress images from a ROS 2 topic.')
    parser.add_argument('--topic_name', type=str, required=True, help='Name of the image topic to subscribe to')
    args = parser.parse_args()

    main(args)

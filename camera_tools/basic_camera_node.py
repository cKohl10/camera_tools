#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import argparse
from typing import Optional


class BasicCameraNode(Node):
    def __init__(self, camera_index: int = 0, publish_rate: float = 30.0):
        super().__init__('basic_camera_node')
        
        self.camera_index = camera_index
        self.publish_rate = publish_rate
        self.bridge = CvBridge()
        
        # Initialize camera
        self.camera = cv2.VideoCapture(camera_index)
        if not self.camera.isOpened():
            self.get_logger().error(f'Failed to open camera at index {camera_index}')
            self.get_logger().error(f'Available cameras: {cv2.VideoCapture.getNumberOfCameras()}')
            raise RuntimeError(f'Camera at index {camera_index} not available')
        
        # Get camera properties
        self.frame_width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.fps = self.camera.get(cv2.CAP_PROP_FPS)
        
        self.get_logger().info(f'Camera initialized: {self.frame_width}x{self.frame_height} @ {self.fps} FPS')
        
        # Create publisher
        self.publisher = self.create_publisher(
            Image,
            f'/camera_{camera_index}/image_raw',
            10
        )
        
        # Create timer for publishing
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_frame)
        
        self.get_logger().info(f'Basic camera node started. Publishing to /camera_{camera_index}/image_raw at {publish_rate} Hz')

    def publish_frame(self):
        """Capture and publish a frame from the camera"""
        ret, frame = self.camera.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame from camera')
            return
        
        try:
            # Convert OpenCV image to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = f'camera_{self.camera_index}'
            
            # Publish the image
            self.publisher.publish(ros_image)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing frame: {str(e)}')

    def cleanup(self):
        """Clean up camera resources"""
        if self.camera.isOpened():
            self.camera.release()
        cv2.destroyAllWindows()
        self.get_logger().info('Camera resources cleaned up')


def main(args=None):
    rclpy.init(args=args)
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='ROS2 Basic Camera Publisher')
    parser.add_argument('-c', '--camera', type=int, default=0, help='Camera index (default: 0)')
    parser.add_argument('-r', '--rate', type=float, default=30.0, help='Publishing rate in Hz (default: 30.0)')
    
    # Filter out ROS2 specific arguments before parsing
    import sys
    filtered_args = []
    i = 0
    while i < len(sys.argv):
        if sys.argv[i] == '--ros-args':
            # Skip --ros-args and all arguments after it
            break
        filtered_args.append(sys.argv[i])
        i += 1
    
    # Parse arguments from filtered command line
    args = parser.parse_args(filtered_args[1:])  # Skip the script name
    
    try:
        # Create and run the node
        node = BasicCameraNode(camera_index=args.camera, publish_rate=args.rate)
        
        # Spin the node
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down gracefully...')
    except Exception as e:
        node.get_logger().error(f'Error: {str(e)}')
    finally:
        if 'node' in locals():
            node.cleanup()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 
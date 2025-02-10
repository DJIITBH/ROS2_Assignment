#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2D

class ObjectSubscriber(Node):
    def __init__(self):
        super().__init__('object_subscriber')
        self.subscription = self.create_subscription(
            Detection2D, '/detected_objects', self.detection_callback, 10)

    def detection_callback(self, msg):
        bbox = msg.bbox
        class_label = msg.results[0].id
        confidence = msg.results[0].score
        self.get_logger().info(f'Detected {class_label} with confidence {confidence:.2f} at ({bbox.center.position.x}, {bbox.center.position.y})')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
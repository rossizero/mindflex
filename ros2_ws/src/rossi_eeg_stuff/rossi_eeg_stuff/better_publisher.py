#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from eeg_msgs.msg import Mindflex


class RepublishNode(Node):
    def __init__(self):
        super().__init__('better_publisher')

        self.subscription = self.create_subscription(String, 'mindflex_raw', self.listener_callback, 10)
        self.publisher = self.create_publisher(Mindflex, 'mindflex', 10)

    def listener_callback(self, msg):
        data = msg.data.split(",")
        mindflex_msg = Mindflex()
        mindflex_msg.signal_quality = int(data[0])
        mindflex_msg.attention = int(data[1])
        mindflex_msg.meditation = int(data[2])
        mindflex_msg.eeg_power = [int(i) for i in data[3:8]]    # Republish the same message
        self.publisher.publish(mindflex_msg)


def main(args=None):
    rclpy.init(args=args)

    node = RepublishNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

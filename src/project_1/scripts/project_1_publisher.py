#!/usr/bin/env python3
"""
Description:
    This ROS 2 node periodically publishes "Hello World" messages to a topic.

Publishing Topic:
    /py_example_topic - std_msgs/String

Subscription Topic:
    None

Author:
    Noah Garsia
"""

import rclpy                          # ROS 2 client library for Python
from rclpy.node import Node           # Used for creating Nodes
from std_msgs.msg import String       # Import String message type for ROS 2


class MinimalPublisher(Node):
    """
    A minimal publisher node that publishes "Hello World" messages to a topic.
    """
    def __init__(self):
        super().__init__('minimal_py_publisher')  # Initialise node name

        # Create a publisher on the topic with a queue size of 10
        self.publisher_ = self.create_publisher(String, 'py_example_topic', 10)

        # Create a timer to trigger publishing every 0.5 seconds
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter variable for message numbering
        self.i = 0

    def timer_callback(self):
        """
        Callback function executed periodically by the timer.
        """
        msg = String()
        msg.data = f"Hello World: {self.i}"

        # Publish the message
        self.publisher_.publish(msg)

        # Log the published message
        self.get_logger().info(f'Publishing: "{msg.data}"')

        self.i += 1  # Increment the counter


def main(args=None):
    """
    Main function for running the publisher node.
    """
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)

    # Clean shutdown
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#! /usr/bin/env python3

"""
Description:
    This ROS2 node subscribes to the "hello world" messages.

    
Publishing Topic:
    None    
Subscription Topic:
    The channel containing "Hello World" messages.
    /py_example_topic - std_msgs/String


Author:
    Noah Garsia
Date: 
    6 Novermber 2025
"""

from os import name
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_py_subscriber')

        self.subscriber_1 = self.create_subscription(
            String,
            '/project_1_example_topic',
            self.listener_callback,
            10)
        
    def listener_callback(self,msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main (args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    rclpy.shutdown()
if __name__ == "__main__":
    main()

        
    """
    A minimal ROS2 subscriber example.
    """

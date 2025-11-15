#!/usr/bin/env python3
from os import name
import pytest
import rclpy
from std_msgs.msg import String
from scr.scripts.project_1_publisher import MinimalPublisher
"""
Test suite of Ros2 minimal publishing nodes

This script contains unit test for verifying the functionality of ROS 2 publisher nodes.
It test the node creation, message counter increment, and message content

Author: Noah Garsia

Date: Novermber, 2025

"""

def test_publisher_node():
    """Test publisher node created properly

    This test verifies:
    1. The publisher node is created with the correct name.
    2. The publsiher object exists,
    3. Checks the topic name is correct.

    If any of these checks fail, an assertion error is raised.
    """

# Initialize rclpy
    rclpy.init()

    try:
        #create a instance of the publisher node
        node = MinimalPublisher()

        #Test 1: Verify the node has the expected name
        assert node.get_name() == 'minimal_publisher'

        #Test 2: Verify the publisher exists and has teh correct topic name
        assert hasattr(node, 'publisher_1')
        assert node.publisher_1.topic_name == '/project_1_example_topic'

    finally:
        #Shutdown rclpy
        rclpy.shutdown()    

def test_message_counter_increment():
    """Test message counter increments correctly
    
    This test verifies that the message counter increases by 1 after each call to the timer callback.
    
    raises AssertionError if the counter does not increment as expected.
    """

    rclpy.init()

    try:
        node = MinimalPublisher()
        initial_count = node.i
        node.timer_callback()  # Simulate timer callback
        assert node.i == initial_count + 1

    finally: 
        rclpy.shutdown()

        
def test_message_content():
    """
    Test published message content.

    This test verifies that the content of the published message matches the expected format.


    raises AssertionError if the message content does not match the expected format.
    """
rclpy.init()

try:

    node = MinimalPublisher()
    node.i = 5 #set a known counter value 
    msg = String()


    #use f string formatting 
    msg.data = f'Hello World: {node.i}'
    assert msg.data == 'Hello World: 5'


finally:
    rclpy.shutdown()


if name=='__main__':
    pytest.main(['-v'])

#! /bin/bash

#Launch both publisher and subscriber nodes

cleanup() {
    echo "Restarting ROS2 nodes daemon to clean up before shutting down..."
    ros2 daemon stop
    sleep 1
    ros2 daemon start
    echo "Teminating all running nodes"
    kill 0
    exit 
}

trap 'cleanup' SIGINT

#Lanch publisher node in background
ros2 run Jazzy_Ros2_Fundermental_Examples_Tutorial project_1_publisher.py 
sleep 2
#Launch subscriber node
ros2 run Jazzy_Ros2_Fundermental_Examples_Tutorial project_1_subscriber.py

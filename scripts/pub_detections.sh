#!/bin/bash

ros2 topic pub -r 10 /tag_detections apriltag_ros_interfaces/msg/AprilTagDetectionArray "{header: 'auto'}"

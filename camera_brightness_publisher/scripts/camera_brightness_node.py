#!/usr/bin/env python3
"""
camera_brightness_node.py

Reads images from the robot's camera (via OpenCV), calculates an appropriate
light level (0..1) to reach a target brightness, and publishes that on a topic.
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import time

from std_msgs.msg import Float32

# Target brightness in [0..255] (for a grayscale mean)
TARGET_BRIGHTNESS = 75.0

# How often to update (seconds)
UPDATE_INTERVAL = 3.0

# Proportional factor to convert brightness error → desired light level
ADJUSTMENT_FACTOR = 0.01

class CameraBrightnessNode(Node):
    def __init__(self):
        super().__init__('camera_brightness_node')

        # Publisher for desired light level (0..1)
        self.pub = self.create_publisher(Float32, 'desired_light_level', 10)

        # Attempt to open default camera
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Error: Cannot access the camera.")
            raise RuntimeError("Camera not accessible.")

        # Start with some midpoint light level
        self.current_light_level = 0.5

        # Keep track of last update time
        self.last_update_time = time.time()

    def spin_camera_loop(self):
        """
        Continuously capture frames, compute brightness, decide new light level,
        and publish it on /desired_light_level.
        """
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("Error: Failed to grab frame from camera.")
                break

            # Calculate mean brightness
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            current_brightness = np.mean(gray)

            # Check if enough time has passed to publish update
            now = time.time()
            if now - self.last_update_time >= UPDATE_INTERVAL:
                # Adjust self.current_light_level based on brightness difference
                brightness_diff = TARGET_BRIGHTNESS - current_brightness
                adjustment = brightness_diff * ADJUSTMENT_FACTOR

                self.current_light_level += adjustment
                # Clamp to [0..1]
                self.current_light_level = max(0.0, min(1.0, self.current_light_level))

                # Publish
                msg = Float32()
                msg.data = float(self.current_light_level)
                self.pub.publish(msg)

                self.get_logger().info(
                    f"Publishing desired light level: {self.current_light_level:.2f} "
                    f"(camera brightness={current_brightness:.2f})"
                )

                self.last_update_time = now

            # (Optional) show the feed
            cv2.imshow("Camera Feed", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraBrightnessNode()
        node.spin_camera_loop()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

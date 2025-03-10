#!/usr/bin/env python3
"""
auto_light_adjust.py

This script captures frames from the robot’s camera (via OpenCV),
calculates brightness, and calls the 'light_control' service to
increase/decrease light brightness via RC override.
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import time

# Import the custom service definition from your package
from bluerov2_control.srv import SetLight

# Target brightness (in grayscale mean, roughly 0-255)
TARGET_BRIGHTNESS = 75.0

# How often to check and adjust lights (seconds)
UPDATE_INTERVAL = 3.0

# Proportional factor for converting brightness error → light level change
ADJUSTMENT_FACTOR = 0.01  # Fine-tune this as needed

class AutoLightAdjustNode(Node):
    """
    ROS2 node that:
      1) Opens the camera with OpenCV,
      2) Periodically measures brightness,
      3) Calls the 'light_control' service to adjust light PWM.
    """

    def __init__(self):
        super().__init__('auto_light_adjust')
        # Create a client for the SetLight service
        self.client = self.create_client(SetLight, 'light_control')

        # Wait until the 'light_control' service is available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("'light_control' service not available. Waiting...")

        # Current "desired" brightness level (0.0 to 1.0) we send to the lights
        self.current_light_level = 0.5

        # Open the default camera (0). Change if your camera index/device is different
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Error: Cannot access the camera.")
            raise RuntimeError("Camera not accessible.")

        # Track time for the next brightness adjustment
        self.last_update_time = time.time()

    def spin_camera_loop(self):
        """
        Main loop that reads frames, computes brightness, and calls service periodically.
        """
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("Error: Failed to grab frame from camera.")
                break

            # Calculate mean brightness (grayscale average)
            current_brightness = self.calculate_brightness(frame)

            # Check if it's time to update the lights
            now = time.time()
            if now - self.last_update_time >= UPDATE_INTERVAL:
                self.adjust_lights(current_brightness)
                self.last_update_time = now

            # Display the camera feed (optional)
            cv2.imshow("Camera Feed", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # Allow ROS2 events (e.g., service responses) to be processed
            rclpy.spin_once(self, timeout_sec=0.01)

        self.cap.release()
        cv2.destroyAllWindows()

    def calculate_brightness(self, frame):
        """
        Compute the average brightness of the image.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return np.mean(gray)

    def adjust_lights(self, current_brightness):
        """
        Compute how much to adjust self.current_light_level based on
        how far the camera brightness is from TARGET_BRIGHTNESS,
        then call the 'light_control' service.
        """
        brightness_diff = TARGET_BRIGHTNESS - current_brightness
        # Convert difference in [0..255] space to a 0..1 light-level change
        adjustment = brightness_diff * ADJUSTMENT_FACTOR

        # Update our light level (0.0 = off, 1.0 = full brightness)
        self.current_light_level += adjustment
        self.current_light_level = max(0.0, min(1.0, self.current_light_level))

        self.get_logger().info(
            f"\nCurrent camera brightness: {current_brightness:.2f}, "
            f"setting light level: {self.current_light_level:.2f}"
        )

        # Call the SetLight service with the new brightness
        request = SetLight.Request()
        request.data = float(self.current_light_level)

        self.future = self.client.call_async(request)
        # Response is handled asynchronously, but for simple usage we don't strictly need to wait

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AutoLightAdjustNode()
        node.spin_camera_loop()
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        print(f"Runtime error: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

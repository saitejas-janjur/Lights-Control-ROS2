#!/usr/bin/env python3
# publish_light_recommendation.py

import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import Float32

# Desired brightness in [0..255]
TARGET_BRIGHTNESS = 75

class PublishLightRecommendationNode(Node):
    def __init__(self):
        super().__init__('publish_light_recommendation_node')

        # Subscribe to the camera feed on /camera/image (bgr8 encoding)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10
        )

        # Publisher for recommended brightness (0..1), under mavros namespace
        self.brightness_pub = self.create_publisher(Float32, '/mavros/light_brightness', 10)

        # Store the most recent measured brightness (0..255)
        self.current_brightness = 0.0

        # Initialize the recommended light setting (0..1)
        self.current_recommendation = 0.5

        # Create a timer that fires every 3 seconds to compute & publish
        self.timer_period = 3.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info("Publish Light Recommendation Node started.")

    def image_callback(self, msg: Image):
        """
        Called whenever a new image is received.
        We'll parse the BGR data with NumPy and compute the brightness,
        storing the result for the next timer cycle.
        """
        # Convert msg.data (bytes) → NumPy array
        bgr_array = np.frombuffer(msg.data, dtype=np.uint8)

        # Reshape to (height, width, 3) if valid
        if msg.height > 0 and msg.width > 0:
            bgr_array = bgr_array.reshape((msg.height, msg.width, 3))
        else:
            self.get_logger().error("Invalid image dimensions.")
            return

        # Approximate grayscale brightness: 0.114B + 0.587G + 0.299R
        blue  = bgr_array[:, :, 0].astype(np.float32)
        green = bgr_array[:, :, 1].astype(np.float32)
        red   = bgr_array[:, :, 2].astype(np.float32)
        gray = 0.114 * blue + 0.587 * green + 0.299 * red

        # Store the average brightness for the timer
        self.current_brightness = float(np.mean(gray))

    def timer_callback(self):
        """
        Fired every 3 seconds, uses the latest brightness reading 
        to compute a recommended light level proportionally and publish it.
        """
        # Current brightness last computed in image_callback
        diff = TARGET_BRIGHTNESS - self.current_brightness

        # Proportional factor (tune as needed)
        adjustment = (diff / 255.0) * 0.5

        # Calculate new recommendation in [0..1]
        new_setting = self.current_recommendation + adjustment
        new_setting = min(max(new_setting, 0.0), 1.0)

        self.current_recommendation = new_setting
        self.get_logger().info(
            f"[Timer] Measured brightness={self.current_brightness:.2f}, "
            f"Recommended light={new_setting:.2f}"
        )

        # Publish the new recommendation on /mavros/light_brightness
        msg = Float32()
        msg.data = float(new_setting)
        self.brightness_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PublishLightRecommendationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

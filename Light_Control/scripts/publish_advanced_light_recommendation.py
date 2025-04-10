#!/usr/bin/env python3
# publish_advanced_light_recommendation.py

import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import Float32

# Desired brightness in [0..255]
TARGET_BRIGHTNESS = 75

# Weighting factors for how much each metric influences final recommendation
ALPHA_BRIGHTNESS = 0.70  # e.g., 70% weight on brightness
BETA_CONTRAST    = 0.20  # e.g., 20% weight on contrast
GAMMA_SHARPNESS  = 0.10  # e.g., 10% weight on sharpness

class PublishAdvancedLightRecommendationNode(Node):
    def __init__(self):
        super().__init__('publish_advanced_light_recommendation_node')

        # Subscribe to camera feed on /camera/image (bgr8 encoding)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10
        )

        # Publisher for final recommended brightness [0..1], under mavros
        self.brightness_pub = self.create_publisher(Float32, '/mavros/light_brightness', 10)

        # Store metrics
        self.current_brightness = 0.0  # average grayscale brightness [0..255]
        self.current_contrast   = 0.0  # simple contrast metric
        self.current_sharpness  = 0.0  # simple sharpness metric

        # We'll track a baseline recommended brightness from the prior cycle
        self.current_recommendation = 0.5

        # Create a timer every 3 seconds
        self.timer_period = 3.0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info("Publish Advanced Light Recommendation Node started.")

    def image_callback(self, msg: Image):
        """
        Called whenever a new image is received.
        We'll parse the BGR data, compute brightness, contrast, and sharpness,
        and store them for the next timer callback.
        """
        # Convert raw bytes to a NumPy array
        bgr_array = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.height > 0 and msg.width > 0:
            bgr_array = bgr_array.reshape((msg.height, msg.width, 3))
        else:
            self.get_logger().error("Invalid image dimensions.")
            return

        # ---- 1) BRIGHTNESS ----
        # Approximate grayscale: 0.114B + 0.587G + 0.299R
        blue  = bgr_array[:, :, 0].astype(np.float32)
        green = bgr_array[:, :, 1].astype(np.float32)
        red   = bgr_array[:, :, 2].astype(np.float32)
        gray = 0.114 * blue + 0.587 * green + 0.299 * red
        self.current_brightness = float(np.mean(gray))

        # ---- 2) CONTRAST ----
        # A simple measure: the std deviation of grayscale
        # Larger std => higher contrast, smaller => lower contrast
        self.current_contrast = float(np.std(gray))

        # ---- 3) SHARPNESS ----
        # A naive Laplacian-based sharpness measure using a simple kernel
        # We'll do a small convolution here in pure NumPy for demonstration.
        # (For a real system, consider using OpenCV or a specialized library
        #  for performance and correct handling of borders.)
        laplacian_kernel = np.array([[0,  1, 0],
                                     [1, -4, 1],
                                     [0,  1, 0]], dtype=np.float32)

        # We'll skip complex border handling and do a simple valid-convolution approach
        # for demonstration. If the image is large, this is workable; for small images
        # or performance-critical code, consider more robust solutions.
        # We'll define a small helper function:
        self.current_sharpness = self.compute_laplacian_sharpness(gray, laplacian_kernel)

    def compute_laplacian_sharpness(self, gray_image, kernel):
        """Convolves 'gray_image' with 'kernel' in a naive way, then
        returns an averaged absolute response as a 'sharpness' metric."""
        # Basic dimensions
        H, W = gray_image.shape
        kH, kW = kernel.shape
        outH = H - kH + 1
        outW = W - kW + 1
        if outH <= 0 or outW <= 0:
            # If the image is smaller than the kernel, fallback
            return 0.0
        
        # Allocate output array for convolution
        laplacian_response = np.zeros((outH, outW), dtype=np.float32)

        # Naive convolution: for each pixel in valid region
        for r in range(outH):
            for c in range(outW):
                # region from (r:r+kH, c:c+kW)
                patch = gray_image[r:r+kH, c:c+kW]
                # sum(patch * kernel)
                value = np.sum(patch * kernel)
                laplacian_response[r, c] = value

        # "Sharpness" ~ average absolute value
        sharpness_metric = float(np.mean(np.abs(laplacian_response)))
        return sharpness_metric

    def timer_callback(self):
        """
        Every 3 seconds, combine brightness, contrast, and sharpness
        into one recommended brightness value [0..1], and publish it.
        """
        # 1) Start with a brightness-based recommendation
        diff = TARGET_BRIGHTNESS - self.current_brightness
        brightness_adjustment = (diff / 255.0) * 0.5
        brightness_reco = self.current_recommendation + brightness_adjustment

        # 2) Incorporate contrast
        # For example, if contrast is *too low*, we might want to
        # increase brightness to bring out details. If contrast is
        # extremely high, we might reduce brightness, etc.
        # We'll define "contrast_factor" as a normalized quantity:
        # e.g., contrast / 64.0 => Just a placeholder scale
        contrast_factor = self.current_contrast / 64.0

        # If you want to invert the effect (low contrast => more brightness),
        # you can do something like: contrast_adjustment = (0.5 - contrast_factor)
        # For demonstration, let's do something naive:
        contrast_adjustment = 0.5 - contrast_factor

        # 3) Incorporate sharpness
        # We'll define "sharpness_factor" as the average Laplacian response / 50
        # just as a placeholder scale
        sharpness_factor = self.current_sharpness / 50.0
        # You might interpret "higher sharpness => we can reduce the light a bit"
        # or the inverse. Let's just do something naive:
        sharpness_adjustment = 0.5 - sharpness_factor

        # Combine them with weighting
        combined_adjustment = (
            ALPHA_BRIGHTNESS * brightness_adjustment +
            BETA_CONTRAST    * contrast_adjustment +
            GAMMA_SHARPNESS  * sharpness_adjustment
        )

        # The final recommended setting is the old setting + combined
        new_setting = self.current_recommendation + combined_adjustment

        # Clamp to [0..1]
        new_setting = min(max(new_setting, 0.0), 1.0)

        # Update internal state
        self.current_recommendation = new_setting

        self.get_logger().info(
            f"[Timer] brightness={self.current_brightness:.2f}, "
            f"contrast={self.current_contrast:.2f}, "
            f"sharpness={self.current_sharpness:.2f}, "
            f"-> recommended={new_setting:.2f}"
        )

        # Publish on /mavros/light_brightness
        msg = Float32()
        msg.data = float(new_setting)
        self.brightness_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PublishAdvancedLightRecommendationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

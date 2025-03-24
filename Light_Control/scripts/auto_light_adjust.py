#!/usr/bin/env python3
# auto_light_adjustment_node_no_opencv.py

import rclpy
from rclpy.node import Node
import numpy as np
import time

from sensor_msgs.msg import Image
from bluerov2_control.srv import SetLight

# Desired brightness in [0..255] (typical for an 8-bit grayscale)
TARGET_BRIGHTNESS = 75

# Update interval in seconds
UPDATE_INTERVAL = 3.0

class AutoBrightnessControlNodeNoOpenCV(Node):
    def __init__(self):
        super().__init__('auto_brightness_control_no_opencv_node')

        # Subscribe to the camera feed on /camera/image (bgr8 encoding)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Create a client for calling the light_control service
        self.client = self.create_client(SetLight, 'light_control')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /light_control service...')

        self.last_update_time = time.time()
        # Start with a mid‐range light level
        self.current_light_setting = 0.5

        self.get_logger().info("Auto Brightness Control (No OpenCV) Node started.")

    def image_callback(self, msg: Image):
        """
        Called whenever a new image is received.
        We'll parse the BGR data manually with NumPy, compute brightness,
        and adjust lights if needed.
        """
        # 1) Convert msg.data (bytes) → NumPy array
        #    BGR8 means each pixel = 3 bytes: [Blue, Green, Red]
        #    The total size = height * width * 3.
        bgr_array = np.frombuffer(msg.data, dtype=np.uint8)

        # 2) Reshape to (height, width, 3)
        if msg.height > 0 and msg.width > 0:
            bgr_array = bgr_array.reshape((msg.height, msg.width, 3))
        else:
            self.get_logger().error("Invalid image dimensions.")
            return

        # 3) Compute brightness.
        #    Option A: simple average of all channels
        #       brightness = np.mean(bgr_array)
        #
        #    Option B: approximate 'grayscale' brightness using a standard formula:
        #       gray = 0.114B + 0.587G + 0.299R
        #       brightness = average of all gray pixels
        #    We'll use Option B:
        blue  = bgr_array[:, :, 0].astype(np.float32)
        green = bgr_array[:, :, 1].astype(np.float32)
        red   = bgr_array[:, :, 2].astype(np.float32)
        gray = 0.114 * blue + 0.587 * green + 0.299 * red
        current_brightness = np.mean(gray)

        # Check if enough time has passed to adjust lights
        now = time.time()
        if (now - self.last_update_time) >= UPDATE_INTERVAL:
            self.adjust_lights(current_brightness)
            self.last_update_time = now

    def adjust_lights(self, current_brightness):
        """
        A basic proportional controller to nudge the light setting [0..1]
        toward TARGET_BRIGHTNESS.
        """
        diff = TARGET_BRIGHTNESS - current_brightness

        # We can scale the difference by 1/255 to normalize, then multiply
        # by some factor (e.g., 0.5) to prevent huge jumps.
        adjustment = (diff / 255.0) * 0.5

        # Update our stored light setting, clamping to [0..1].
        new_setting = self.current_light_setting + adjustment
        new_setting = min(max(new_setting, 0.0), 1.0)
        self.current_light_setting = new_setting

        self.get_logger().info(
            f"Current brightness={current_brightness:.2f}, "
            f"New light setting={new_setting:.2f}"
        )

        # Call the existing service to set the new brightness
        self.call_light_service(new_setting)

    def call_light_service(self, brightness_value):
        """
        Sends a request to the /light_control service with brightness [0..1].
        """
        request = SetLight.Request()
        request.data = float(brightness_value)

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            resp = future.result()
            if resp.success:
                self.get_logger().info(f"Light service responded: {resp.message}")
            else:
                self.get_logger().warn(f"Light service reported failure: {resp.message}")
        else:
            self.get_logger().error("Light service call failed with no result.")

def main(args=None):
    rclpy.init(args=args)
    node = AutoBrightnessControlNodeNoOpenCV()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

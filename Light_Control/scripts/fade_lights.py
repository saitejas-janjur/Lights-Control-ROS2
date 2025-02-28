#!/usr/bin/env python3
# fade_lights.py

import rclpy
from rclpy.node import Node
from bluerov2_control.srv import SetLight
import time

class LightFader(Node):
    def __init__(self):
        super().__init__('light_fader')

        # Create client for SetLight service
        self.client = self.create_client(SetLight, 'light_control')

        # Wait for the service to become available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for the light_control service...")

        # Once service is available, run our fade logic
        self.fade_lights()

    def fade_lights(self):
        """
        Over a total of 3 seconds, fade from brightness=0 -> 1, then back to 0.
        1.5 sec up, 1.5 sec down, in small increments.
        """
        up_down_steps = 30  # number of discrete steps each way
        up_down_time = 1.5  # 1.5 seconds up, 1.5 seconds down
        step_time = up_down_time / up_down_steps  # time per step

        # Fade up from 0.0 to 1.0
        for i in range(up_down_steps + 1):
            brightness = i / up_down_steps  # goes 0.0 .. 1.0
            self.set_brightness(brightness)
            time.sleep(step_time)

        # Fade down from 1.0 to 0.0
        for i in range(up_down_steps, -1, -1):
            brightness = i / up_down_steps  # goes 1.0 .. 0.0
            self.set_brightness(brightness)
            time.sleep(step_time)

        self.get_logger().info("Fade cycle complete. Shutting down.")
        rclpy.shutdown()  # optionally shut down the node

    def set_brightness(self, value):
        """Send one SetLight request to the service (non-blocking call)."""
        request = SetLight.Request()
        request.data = float(value)

        # Call asynchronously
        self.client.call_async(request)

        self.get_logger().info(f"Sent brightness: {value:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = LightFader()
    # This node will do its fade_lights() logic in the constructor,
    # and then calls rclpy.shutdown() upon completion, so no spin needed.
    rclpy.spin(node)


if __name__ == '__main__':
    main()

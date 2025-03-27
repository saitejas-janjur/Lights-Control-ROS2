#!/usr/bin/env python3
# auto_light_adjust.py (with PID control)

import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import Image
from bluerov2_control.srv import SetLight

# Desired brightness in [0..255]
TARGET_BRIGHTNESS = 75

# ======= PID GAINS =======
Kp = 0.10
Ki = 0.02
Kd = 0.05
# =========================

# Integral windup clamp (prevent integral from growing unbounded)
INTEGRAL_MIN = -300.0
INTEGRAL_MAX =  300.0

# Timer period in seconds
TIMER_PERIOD = 1.0

class AutoBrightnessControlPIDNode(Node):
    def __init__(self):
        super().__init__('auto_brightness_control_pid_node')

        # Subscribe to the camera feed on /camera/image (bgr8 encoding)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10
        )

        # Create a client for calling the SetLight service
        self.client = self.create_client(SetLight, 'light_control')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /light_control service...')

        # Initialize brightness reading
        self.current_brightness = 0.0

        # Initialize the PID terms
        self.integral_error = 0.0
        self.prev_error = 0.0

        # Start halfway
        self.current_light_setting = 0.5
        self.send_light_command(self.current_light_setting)

        # Create a timer that fires every 3 seconds to adjust lights
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

        self.get_logger().info("Auto Brightness Control (PID) Node started.")

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

        # Store the average brightness
        self.current_brightness = float(np.mean(gray))

    def timer_callback(self):
        """
        Fired every 3 seconds. Uses PID to adjust the light level
        based on the difference between target and measured brightness.
        """
        # Current error
        error = TARGET_BRIGHTNESS - self.current_brightness

        # -- Proportional term --
        P_term = Kp * error

        # -- Integral term (with basic anti-windup clamp) --
        self.integral_error += error * TIMER_PERIOD
        # clamp the integral to prevent runaway
        if self.integral_error > INTEGRAL_MAX:
            self.integral_error = INTEGRAL_MAX
        elif self.integral_error < INTEGRAL_MIN:
            self.integral_error = INTEGRAL_MIN
        I_term = Ki * self.integral_error

        # -- Derivative term --
        derivative = (error - self.prev_error) / TIMER_PERIOD
        D_term = Kd * derivative

        # Sum the PID terms
        pid_output = P_term + I_term + D_term

        # Update for next cycle
        self.prev_error = error

        # Current_light_setting is what we want in [0..1].
        # We'll interpret pid_output as an *increment* or *delta* for the current setting
        # (or you could interpret it as an absolute fraction directly).
        new_setting = self.current_light_setting + (pid_output / 255.0)

        # Make sure it's in [0, 1]
        new_setting = min(max(new_setting, 0.0), 1.0)

        # Only send an update if there's a noticeable change
        if abs(new_setting - self.current_light_setting) > 1e-3:
            self.current_light_setting = new_setting
            self.get_logger().info(
                f"[Timer] Measured={self.current_brightness:.2f}, "
                f"Error={error:.2f}, "
                f"PID_out={pid_output:.2f}, "
                f"Light={new_setting:.2f}"
            )
            self.send_light_command(new_setting)

    def send_light_command(self, brightness_value: float):
        """
        Send a request to the /light_control service to set brightness in [0..1].
        """
        request = SetLight.Request()
        request.data = brightness_value

        future = self.client.call_async(request)
        future.add_done_callback(self.light_service_response)

    def light_service_response(self, future):
        """Log success or failure of the service call."""
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info(f"Light service responded: {resp.message}")
            else:
                self.get_logger().warn(f"Light service reported failure: {resp.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def turn_off_lights(self):
        """
        Call this to turn off the lights (brightness=0.0) before shutting down.
        """
        self.get_logger().info("Turning off lights before shutdown...")
        self.send_light_command(0.0)

def main(args=None):
    rclpy.init(args=args)
    node = AutoBrightnessControlPIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down...")
    finally:
        # Turn off lights
        node.turn_off_lights()
        # Allow some time (or spin) for the service call to go through
        rclpy.spin_once(node, timeout_sec=1.0)

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# light_service_node.py

import rclpy
from rclpy.node import Node

# Import your newly generated service from the same package
from bluerov2_control.srv import SetFloat

from mavros_msgs.msg import OverrideRCIn

class LightControlService(Node):
    def __init__(self):
        super().__init__('light_control_service')

        # Declare parameters with default values
        self.declare_parameter('light_channel', 9)
        self.declare_parameter('light_min_pwm', 1100)
        self.declare_parameter('light_max_pwm', 1900)  # or 2000, depending on your hardware

        # Retrieve parameters
        self.light_channel = self.get_parameter('light_channel').get_parameter_value().integer_value
        self.light_min_pwm = self.get_parameter('light_min_pwm').get_parameter_value().integer_value
        self.light_max_pwm = self.get_parameter('light_max_pwm').get_parameter_value().integer_value

        # Create the service using our new SetFloat type
        self.srv = self.create_service(SetFloat, 'light_control', self.handle_light_control)
        self.get_logger().info("Light control service (SetFloat) is ready.")

        # Publisher to /mavros/rc/override
        self.rc_override_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)

        # Initialize RC override message (no override for all channels)
        self.rc_override_msg = OverrideRCIn()
        self.rc_override_msg.channels = [0] * 18

    def handle_light_control(self, request, response):
        """
        Service callback for setting light brightness [0..100].
        If 0 -> off, if 1..100 -> scaled brightness.
        """
        brightness = request.data  # float in [0..100], though we clamp below if outside

        # Clamp brightness
        if brightness < 0.0:
            brightness = 0.0
        elif brightness > 100.0:
            brightness = 100.0

        # Compute PWM from brightness
        # 0% -> light_min_pwm, 100% -> light_max_pwm
        pwm_value = int(self.light_min_pwm + (brightness / 100.0) * (self.light_max_pwm - self.light_min_pwm))

        # Override channel
        self.override_rc(self.light_channel, pwm_value)

        # Populate response
        response.success = True
        if brightness == 0.0:
            response.message = f"Light turned off (PWM={pwm_value})."
        else:
            response.message = f"Light set to {brightness}% brightness (PWM={pwm_value})."

        self.get_logger().info(response.message)
        return response

    def override_rc(self, channel, value):
        """Publish one RC override message to set the PWM on a single channel."""
        if not (1 <= channel <= 18):
            self.get_logger().error(f"Invalid RC channel: {channel}. Must be between 1 and 18.")
            return
        
        self.rc_override_msg.channels[channel - 1] = value

        # One-shot publish
        self.rc_override_pub.publish(self.rc_override_msg)
        self.get_logger().debug(f"Published RC Override on channel {channel}: {value}")

        # Optional: revert to 0 immediately so we don't block other inputs
        # self.rc_override_msg.channels[channel - 1] = 0
        # self.rc_override_pub.publish(self.rc_override_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LightControlService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Light control service stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

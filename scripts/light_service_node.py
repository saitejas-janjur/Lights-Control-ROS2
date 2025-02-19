#!/usr/bin/env python3
# light_service_node.py

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.clock import Clock
from std_srvs.srv import SetBool
from mavros_msgs.msg import OverrideRCIn
import time


class LightControlService(Node):
    def __init__(self):
        super().__init__('light_control_service')

        # Declare parameters with default values
        self.declare_parameter('light_channel', 9)
        self.declare_parameter('light_on_value', 1500)
        self.declare_parameter('light_off_value', 1100)
        self.declare_parameter('override_duration', 1.0)  # in seconds

        # Retrieve parameters
        self.light_channel = self.get_parameter('light_channel').get_parameter_value().integer_value
        self.light_on_value = self.get_parameter('light_on_value').get_parameter_value().integer_value
        self.light_off_value = self.get_parameter('light_off_value').get_parameter_value().integer_value
        self.override_duration = self.get_parameter('override_duration').get_parameter_value().double_value

        # Create the service
        self.srv = self.create_service(SetBool, 'light_control', self.handle_light_control)
        self.get_logger().info("Light control service is ready.")

        # Publisher to /mavros/rc/override
        self.rc_override_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)

        # Initialize OverrideRCIn message
        self.rc_override_msg = OverrideRCIn()
        self.rc_override_msg.channels = [0] * 18  # Initialize all channels to 0 (no override)
        
        # We'll only publish overrides for a short period after each request.
        self.override_timer = None
        self.publish_end_time = None
        
    def handle_light_control(self, request, response):
        """
        Service callback for turning the light on/off. 
        Publishes RC override for 'override_duration' seconds, then stops publishing.
        """
        if request.data:
            # Turn light on
            self.override_rc(self.light_channel, self.light_on_value)
            response.success = True
            response.message = "Light turned on"
            self.get_logger().info("Light turned on via RC override.")
        else:
            # Turn light off
            self.override_rc(self.light_channel, self.light_off_value)
            response.success = True
            response.message = "Light turned off"
            self.get_logger().info("Light turned off via RC override.")

        # Start (or restart) the override timer for the specified duration
        self.start_override_timer()
        return response

    def override_rc(self, channel, value):
        """Set the override PWM for a single channel if valid."""
        if not (1 <= channel <= 18):
            self.get_logger().error(f"Invalid RC channel: {channel}. Must be between 1 and 18.")
            return
        self.rc_override_msg.channels[channel - 1] = value
        self.get_logger().debug(f"Set RC Override on channel {channel} to {value}")

    def start_override_timer(self):
        """
        Start a repeating timer to publish the RC override at 10 Hz for `override_duration` seconds.
        If a timer is already running, we reset it to extend the time.
        """
        # If there's already a timer running, destroy it and start fresh
        if self.override_timer is not None:
            self.destroy_timer(self.override_timer)
            self.override_timer = None

        now = self.get_clock().now()
        self.publish_end_time = now + Duration(seconds=self.override_duration)

        # Create a new timer that fires at 10 Hz
        self.override_timer = self.create_timer(0.1, self.publish_override)

    def publish_override(self):
        """
        Publish the RC Override message. 
        If the current time is past `publish_end_time`, stop publishing and clear override.
        """
        now = self.get_clock().now()
        if now >= self.publish_end_time:
            # Time is up; stop overriding
            self.stop_override()
            return

        self.rc_override_pub.publish(self.rc_override_msg)
        self.get_logger().debug(f"Publishing RC Override: {self.rc_override_msg.channels}")

    def stop_override(self):
        """
        Stop the override timer and clear the RC override channel so as not to block normal controls.
        """
        if self.override_timer is not None:
            self.destroy_timer(self.override_timer)
            self.override_timer = None
        
        # Optionally revert to 0 on the light channel so it's no longer overridden.
        self.rc_override_msg.channels[self.light_channel - 1] = 0
        self.rc_override_pub.publish(self.rc_override_msg)
        self.get_logger().info("Stopped RC override; channel reverted to 0.")

    def destroy(self):
        """Clean up node before shutdown."""
        if self.override_timer is not None:
            self.destroy_timer(self.override_timer)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    light_control_service = LightControlService()
    try:
        rclpy.spin(light_control_service)
    except KeyboardInterrupt:
        light_control_service.get_logger().info("Light control service stopped by user.")
    finally:
        light_control_service.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# light_service_node.py

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from mavros_msgs.msg import OverrideRCIn

class LightControlService(Node):
    def __init__(self):
        super().__init__('light_control_service')

        # Declare parameters with default values
        self.declare_parameter('light_channel', 9)
        self.declare_parameter('light_on_value', 1500)
        self.declare_parameter('light_off_value', 1100)

        # Retrieve parameters
        self.light_channel = self.get_parameter('light_channel').get_parameter_value().integer_value
        self.light_on_value = self.get_parameter('light_on_value').get_parameter_value().integer_value
        self.light_off_value = self.get_parameter('light_off_value').get_parameter_value().integer_value

        # Create the service
        self.srv = self.create_service(SetBool, 'light_control', self.handle_light_control)
        self.get_logger().info("Light control service is ready.")

        # Publisher to /mavros/rc/override
        self.rc_override_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)

        # Initialize a reusable OverrideRCIn message (channels all 0 = no override)
        self.rc_override_msg = OverrideRCIn()
        self.rc_override_msg.channels = [0] * 18

    def handle_light_control(self, request, response):
        """
        Service callback for turning the light on/off. 
        Publishes RC override exactly once and then returns.
        """
        if request.data:
            # Turn light on
            self.override_rc(self.light_channel, self.light_on_value)
            response.success = True
            response.message = "Light turned on (single RC override published)"
            self.get_logger().info("Light turned on via single RC override.")
        else:
            # Turn light off
            self.override_rc(self.light_channel, self.light_off_value)
            response.success = True
            response.message = "Light turned off (single RC override published)"
            self.get_logger().info("Light turned off via single RC override.")
        
        return response

    def override_rc(self, channel, value):
        """
        Set (and publish) a single RC override PWM value for the specified channel.
        Only published once.
        """
        # Validate channel
        if not (1 <= channel <= 18):
            self.get_logger().error(f"Invalid RC channel: {channel}. Must be between 1 and 18.")
            return
        
        # Modify the channel in the override message
        self.rc_override_msg.channels[channel - 1] = value

        # Publish immediately (one-shot)
        self.rc_override_pub.publish(self.rc_override_msg)
        self.get_logger().debug(f"Published single RC Override on channel {channel}: {value}")

        # Optionally revert the channel back to 0 (no override) immediately after:
        # self.rc_override_msg.channels[channel - 1] = 0
        # self.rc_override_pub.publish(self.rc_override_msg)

def main(args=None):
    rclpy.init(args=args)
    light_control_service = LightControlService()
    try:
        rclpy.spin(light_control_service)
    except KeyboardInterrupt:
        light_control_service.get_logger().info("Light control service stopped by user.")
    finally:
        light_control_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

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

# Configuration Parameters
TARGET_BRIGHTNESS = 75.0      # Desired grayscale brightness (0..255)
UPDATE_INTERVAL   = 3.0       # Time interval in seconds between updates

# PID Gains (tune these values to get stable performance)
Kp = 0.01  # Proportional gain
Ki = 0.0005  # Integral gain
Kd = 0.001  # Derivative gain

# For safety, clamp integral to avoid "windup"
INTEGRAL_LIMIT = 1000.0

class AutoLightAdjustNode(Node):
    """
    ROS2 node that:
      1) Opens the camera with OpenCV,
      2) Periodically measures brightness,
      3) Uses a PID controller to compute a new light level (0..1),
      4) Calls the 'light_control' service to apply that light level via RC override.
    """

    def __init__(self):
        super().__init__('auto_light_adjust')

        # Create a client for the SetLight service
        self.client = self.create_client(SetLight, 'light_control')

        # Wait until the 'light_control' service is available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("'light_control' service not available. Waiting...")

        # Open the default camera (index=0)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Error: Cannot access the camera.")
            raise RuntimeError("Camera not accessible.")

        # Current "commanded" light level in [0.0, 1.0]
        # Start with something moderate, e.g. 0.5
        self.current_light_level = 0.5

        # PID State
        self.pid_integral = 0.0       # Integral of the error
        self.pid_last_error = 0.0     # For derivative calculation
        self.last_update_time = time.time()  # For scheduling brightness updates
        self.prev_time = self.last_update_time  # For derivative dt

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
                self.update_pid_control(current_brightness, now)
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
        Compute the average brightness of the image (0..255).
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return np.mean(gray)

    def update_pid_control(self, current_brightness, current_time):
        """
        Perform one cycle of PID control:
          error = (target - measured)
          integral += error * dt
          derivative = (error - last_error) / dt
          output = Kp*error + Ki*integral + Kd*derivative
        Then clamp output to [0..1], call the light_control service.
        """
        # Calculate dt
        dt = current_time - self.prev_time
        self.prev_time = current_time

        # PID Error
        error = TARGET_BRIGHTNESS - current_brightness

        # Integral term
        self.pid_integral += error * dt
        # Clamp integral to avoid windup
        if self.pid_integral > INTEGRAL_LIMIT:
            self.pid_integral = INTEGRAL_LIMIT
        elif self.pid_integral < -INTEGRAL_LIMIT:
            self.pid_integral = -INTEGRAL_LIMIT

        # Derivative term
        derivative = (error - self.pid_last_error) / dt if dt > 0 else 0.0

        # PID Output
        pid_output = (Kp * error) + (Ki * self.pid_integral) + (Kd * derivative)

        # Update "last error"
        self.pid_last_error = error

        # Update current_light_level (0.0..1.0) with PID command
        self.current_light_level += pid_output
        # Clamp to [0, 1]
        if self.current_light_level > 1.0:
            self.current_light_level = 1.0
        elif self.current_light_level < 0.0:
            self.current_light_level = 0.0

        # Log info
        self.get_logger().info(
            f"\nCurrent brightness: {current_brightness:.2f}, Error: {error:.2f}\n"
            f"PID Output: {pid_output:.4f}, Light Level: {self.current_light_level:.2f}"
        )

        # Call the SetLight service with the new brightness
        request = SetLight.Request()
        request.data = float(self.current_light_level)
        self.client.call_async(request)

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
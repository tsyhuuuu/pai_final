#!/usr/bin/env python3
"""
ROS 2 Robot Controller for Nova Carter3
Handles robot movement and camera data processing
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import logging
from typing import Optional, Callable, Dict, Any
import threading
import time

class NovaCarterController(Node):
    def __init__(self, node_name: str = 'nova_carter_controller'):
        super().__init__(node_name)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.left_camera_sub = self.create_subscription(
            Image, '/front_stereo_camera/left/image_raw', 
            self.left_camera_callback, 10)
        self.right_camera_sub = self.create_subscription(
            Image, '/front_stereo_camera/right/image_raw', 
            self.right_camera_callback, 10)
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Image storage
        self.left_image = None
        self.right_image = None
        
        # Movement parameters
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.5  # rad/s
        
        # Safety parameters
        self.min_distance = 0.5  # minimum distance to obstacle
        self.max_speed = 0.5     # maximum linear speed
        
        # State variables
        self.is_moving = False
        self.current_command = None
        
        self.logger = logging.getLogger(__name__)
        self.get_logger().info("Nova Carter Controller initialized")
    
    def left_camera_callback(self, msg: Image):
        """Process left camera image"""
        try:
            self.left_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error processing left camera: {e}")
    
    def right_camera_callback(self, msg: Image):
        """Process right camera image"""
        try:
            self.right_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error processing right camera: {e}")
    
    def get_current_image(self) -> Optional[np.ndarray]:
        """Get current camera image (left camera)"""
        return self.left_image
    
    def move_forward(self, distance: float = 1.0, speed: float = None):
        """Move robot forward by specified distance"""
        if speed is None:
            speed = self.linear_speed
        
        twist = Twist()
        twist.linear.x = min(speed, self.max_speed)
        twist.angular.z = 0.0
        
        # Calculate time to move
        move_time = distance / speed
        
        self.get_logger().info(f"Moving forward {distance}m at {speed}m/s")
        self.is_moving = True
        
        # Start movement
        start_time = time.time()
        while (time.time() - start_time) < move_time:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Stop movement
        self.stop_movement()
    
    def move_backward(self, distance: float = 1.0, speed: float = None):
        """Move robot backward by specified distance"""
        if speed is None:
            speed = self.linear_speed
        
        twist = Twist()
        twist.linear.x = -min(speed, self.max_speed)
        twist.angular.z = 0.0
        
        move_time = distance / speed
        
        self.get_logger().info(f"Moving backward {distance}m at {speed}m/s")
        self.is_moving = True
        
        start_time = time.time()
        while (time.time() - start_time) < move_time:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.stop_movement()
    
    def turn_left(self, angle: float = 90.0, speed: float = None):
        """Turn robot left by specified angle (degrees)"""
        if speed is None:
            speed = self.angular_speed
        
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = speed
        
        # Convert angle to radians and calculate time
        angle_rad = np.radians(angle)
        turn_time = angle_rad / speed
        
        self.get_logger().info(f"Turning left {angle} degrees")
        self.is_moving = True
        
        start_time = time.time()
        while (time.time() - start_time) < turn_time:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.stop_movement()
    
    def turn_right(self, angle: float = 90.0, speed: float = None):
        """Turn robot right by specified angle (degrees)"""
        if speed is None:
            speed = self.angular_speed
        
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -speed
        
        angle_rad = np.radians(angle)
        turn_time = angle_rad / speed
        
        self.get_logger().info(f"Turning right {angle} degrees")
        self.is_moving = True
        
        start_time = time.time()
        while (time.time() - start_time) < turn_time:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.stop_movement()
    
    def stop_movement(self):
        """Stop all robot movement"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        
        self.cmd_vel_pub.publish(twist)
        self.is_moving = False
        self.get_logger().info("Movement stopped")
    
    def move_to_target(self, target_x: float, target_y: float, current_x: float = 0.0, current_y: float = 0.0):
        """Move robot to target position using simple proportional control"""
        
        # Calculate distance and angle to target
        dx = target_x - current_x
        dy = target_y - current_y
        distance = np.sqrt(dx**2 + dy**2)
        target_angle = np.arctan2(dy, dx)
        
        self.get_logger().info(f"Moving to target: ({target_x}, {target_y}), distance: {distance:.2f}m")
        
        # Turn towards target
        current_angle = 0.0  # Assuming robot starts facing forward
        angle_diff = target_angle - current_angle
        
        if abs(angle_diff) > 0.1:  # 0.1 radian tolerance
            turn_angle = np.degrees(angle_diff)
            if turn_angle > 0:
                self.turn_left(turn_angle)
            else:
                self.turn_right(abs(turn_angle))
        
        # Move forward to target
        if distance > 0.1:  # 0.1 meter tolerance
            self.move_forward(distance)
    
    def approach_object(self, object_center_x: int, object_center_y: int, 
                       image_width: int, image_height: int, 
                       target_distance: float = 1.0):
        """Approach detected object using visual feedback"""
        
        # Calculate angular error (object position relative to image center)
        image_center_x = image_width // 2
        angular_error = (object_center_x - image_center_x) / image_width
        
        # Proportional control gains
        angular_gain = 2.0
        linear_gain = 0.5
        
        # Create movement command
        twist = Twist()
        
        # Angular velocity to center object
        twist.angular.z = -angular_gain * angular_error
        
        # Linear velocity to approach (simplified - would need depth estimation)
        twist.linear.x = linear_gain * 0.3  # Constant slow approach
        
        # Limit speeds
        twist.linear.x = max(-self.max_speed, min(self.max_speed, twist.linear.x))
        twist.angular.z = max(-self.angular_speed, min(self.angular_speed, twist.angular.z))
        
        self.get_logger().info(f"Approaching object: angular_error={angular_error:.2f}")
        self.cmd_vel_pub.publish(twist)
    
    def execute_generated_code(self, code: str):
        """Execute generated robot control code with safety restrictions"""
        try:
            # Restricted imports for safety
            safe_globals = {
                '__builtins__': {
                    'len': len, 'range': range, 'min': min, 'max': max,
                    'abs': abs, 'round': round, 'print': print
                },
                'controller': self,
                'rclpy': rclpy,
                'Twist': Twist,
                'np': np,
                'time': time,
                'threading': threading
            }
            
            # Check for dangerous patterns
            dangerous_patterns = ['import', 'exec', 'eval', 'open', 'file', '__import__', 'subprocess']
            for pattern in dangerous_patterns:
                if pattern in code:
                    self.get_logger().error(f"Dangerous pattern detected in code: {pattern}")
                    return
            
            # Execute with restricted globals
            exec(code, safe_globals, {})
            
        except Exception as e:
            self.get_logger().error(f"Error executing generated code: {e}")
    
    def shutdown(self):
        """Shutdown controller safely"""
        self.stop_movement()
        self.get_logger().info("Controller shutdown")
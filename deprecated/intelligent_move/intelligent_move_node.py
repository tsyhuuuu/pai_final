#!/usr/bin/env python3
"""
ROS 2 Node for Intelligent Move System
Combines speech recognition, object detection, and robot control
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import threading
import time
import logging
from typing import Optional, Dict, Any

from intelligent_move.src.speech_recognition_module import SpeechRecognitionModule
from intelligent_move.src.object_detection import RedBlockDetector
from intelligent_move.src.chatgpt_integration import ChatGPTCodeGenerator


class IntelligentMoveNode(Node):
    def __init__(self):
        super().__init__('intelligent_move_node')
        
        # Setup logging
        self.logger = logging.getLogger(__name__)
        self.get_logger().info('Initializing Intelligent Move Node')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.camera_sub = self.create_subscription(
            Image, '/front_stereo_camera/left/image_raw',
            self.camera_callback, 10)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Current camera image
        self.current_image = None
        
        # Initialize components
        self.speech_recognizer = SpeechRecognitionModule()
        self.object_detector = RedBlockDetector()
        
        # Initialize ChatGPT (requires API key)
        api_key = os.getenv('OPENAI_API_KEY')
        if api_key:
            self.code_generator = ChatGPTCodeGenerator(api_key)
            self.chatgpt_enabled = True
        else:
            self.get_logger().warning("OPENAI_API_KEY not set, ChatGPT disabled")
            self.chatgpt_enabled = False
        
        # Movement parameters
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.5  # rad/s
        self.max_speed = 0.5
        
        # State variables
        self.is_moving = False
        self.is_listening = False
        self.last_detection = None
        
        # Services
        self.declare_parameter('mode', 'voice')  # voice, manual, auto
        self.mode = self.get_parameter('mode').value
        
        self.get_logger().info(f'Node initialized in {self.mode} mode')
        
        # Start operation based on mode
        if self.mode == 'voice':
            self.start_voice_control()
        elif self.mode == 'auto':
            self.start_autonomous_mode()
        elif self.mode == 'manual':
            self.start_manual_mode()
    
    def camera_callback(self, msg: Image):
        """Process camera image"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error processing camera image: {e}")
    
    def detect_red_block(self) -> Optional[Dict[str, Any]]:
        """Detect red block in current camera view"""
        if self.current_image is None:
            self.get_logger().warning("No camera image available")
            return None
        
        red_block = self.object_detector.find_closest_red_block(self.current_image)
        
        if red_block:
            x1, y1, x2, y2, confidence = red_block
            center_x, center_y = self.object_detector.get_block_center(red_block)
            
            return {
                'bbox': (x1, y1, x2, y2),
                'confidence': confidence,
                'center': (center_x, center_y),
                'image_width': self.current_image.shape[1],
                'image_height': self.current_image.shape[0]
            }
        
        return None
    
    def move_forward(self, distance: float = 1.0, speed: float = None):
        """Move robot forward"""
        if speed is None:
            speed = self.linear_speed
        
        twist = Twist()
        twist.linear.x = min(speed, self.max_speed)
        twist.angular.z = 0.0
        
        move_time = distance / speed
        
        self.get_logger().info(f"Moving forward {distance}m at {speed}m/s")
        self.is_moving = True
        
        start_time = time.time()
        while (time.time() - start_time) < move_time:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.stop_movement()
    
    def move_backward(self, distance: float = 1.0, speed: float = None):
        """Move robot backward"""
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
        """Turn robot left"""
        if speed is None:
            speed = self.angular_speed
        
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = speed
        
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
        """Turn robot right"""
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
    
    def approach_red_block(self, block_info: Dict[str, Any]):
        """Approach detected red block using visual feedback"""
        center_x, center_y = block_info['center']
        image_width = block_info['image_width']
        image_height = block_info['image_height']
        
        # Calculate angular error
        image_center_x = image_width // 2
        angular_error = (center_x - image_center_x) / image_width
        
        # Proportional control gains
        angular_gain = 2.0
        linear_gain = 0.5
        
        # Create movement command
        twist = Twist()
        twist.angular.z = -angular_gain * angular_error
        twist.linear.x = linear_gain * 0.3  # Slow approach
        
        # Limit speeds
        twist.linear.x = max(-self.max_speed, min(self.max_speed, twist.linear.x))
        twist.angular.z = max(-self.angular_speed, min(self.angular_speed, twist.angular.z))
        
        self.get_logger().info(f"Approaching red block: angular_error={angular_error:.2f}")
        self.cmd_vel_pub.publish(twist)
    
    def process_voice_command(self, command: str):
        """Process voice command"""
        self.get_logger().info(f"Processing command: {command}")
        
        # Detect red block
        block_info = self.detect_red_block()
        
        if block_info:
            if self.chatgpt_enabled:
                # Generate code using ChatGPT
                generated_code = self.code_generator.generate_block_approach_code(block_info)
                
                if generated_code and self.code_generator.validate_generated_code(generated_code):
                    self.get_logger().info("Executing ChatGPT generated code")
                    self.execute_generated_code(generated_code)
                else:
                    self.get_logger().warning("ChatGPT failed, using fallback")
                    self.fallback_approach_block(block_info)
            else:
                self.fallback_approach_block(block_info)
        else:
            # No red block detected, process general movement
            self.fallback_movement(command)
    
    def fallback_approach_block(self, block_info: Dict[str, Any]):
        """Fallback method to approach red block"""
        self.get_logger().info("Using fallback approach for red block")
        
        def approach_thread():
            for _ in range(50):  # Approach for 5 seconds
                if self.current_image is not None:
                    current_block = self.detect_red_block()
                    if current_block:
                        self.approach_red_block(current_block)
                    else:
                        self.stop_movement()
                        break
                time.sleep(0.1)
            self.stop_movement()
        
        thread = threading.Thread(target=approach_thread)
        thread.daemon = True
        thread.start()
    
    def fallback_movement(self, command: str):
        """Fallback movement commands"""
        self.get_logger().info("Using fallback movement commands")
        
        command_lower = command.lower()
        
        if "forward" in command_lower or "ahead" in command_lower:
            self.move_forward(1.0)
        elif "backward" in command_lower or "back" in command_lower:
            self.move_backward(1.0)
        elif "left" in command_lower:
            self.turn_left(45.0)
        elif "right" in command_lower:
            self.turn_right(45.0)
        elif "stop" in command_lower:
            self.stop_movement()
        else:
            self.get_logger().warning(f"Unknown command: {command}")
    
    def execute_generated_code(self, code: str):
        """Execute generated robot control code with safety restrictions"""
        try:
            # Restricted imports for safety
            safe_globals = {
                '__builtins__': {
                    'len': len, 'range': range, 'min': min, 'max': max,
                    'abs': abs, 'round': round, 'print': print
                },
                'node': self,
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
    
    def start_voice_control(self):
        """Start voice control mode"""
        self.get_logger().info("Starting voice control mode")
        self.is_listening = True
        
        def voice_thread():
            while rclpy.ok() and self.is_listening:
                try:
                    command = self.speech_recognizer.listen_for_command(timeout=5)
                    if command:
                        self.process_voice_command(command)
                except Exception as e:
                    self.get_logger().error(f"Voice recognition error: {e}")
                    time.sleep(1)
        
        thread = threading.Thread(target=voice_thread)
        thread.daemon = True
        thread.start()
    
    def start_autonomous_mode(self):
        """Start autonomous red block detection mode"""
        self.get_logger().info("Starting autonomous mode")
        
        def autonomous_thread():
            while rclpy.ok():
                try:
                    block_info = self.detect_red_block()
                    
                    if block_info:
                        self.get_logger().info("Red block detected, approaching...")
                        self.fallback_approach_block(block_info)
                        
                        # Wait for approach to complete
                        time.sleep(3.0)
                        
                        # Check if we're close enough
                        new_block_info = self.detect_red_block()
                        if new_block_info:
                            bbox = new_block_info['bbox']
                            block_area = (bbox[2] - bbox[0]) * (bbox[3] - bbox[1])
                            
                            if block_area > 50000:  # Close enough
                                self.get_logger().info("Successfully reached red block!")
                                self.stop_movement()
                                break
                    else:
                        # Search by rotating
                        self.get_logger().info("Searching for red block...")
                        self.turn_right(30.0)
                        time.sleep(1.0)
                    
                    time.sleep(0.5)
                    
                except Exception as e:
                    self.get_logger().error(f"Autonomous mode error: {e}")
                    time.sleep(1)
        
        thread = threading.Thread(target=autonomous_thread)
        thread.daemon = True
        thread.start()
    
    def start_manual_mode(self):
        """Start manual command mode"""
        self.get_logger().info("Manual mode - use service calls or topic commands")
        # Manual mode would typically use service calls or additional topics
        # For now, just log that it's available
        pass


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = IntelligentMoveNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
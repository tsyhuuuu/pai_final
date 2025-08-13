#!/usr/bin/env python3
"""
ChatGPT Integration Module for ROS 2 Code Generation
Converts natural language commands to ROS 2 control code
"""

import openai
import logging
from typing import Optional, Dict, Any
import json

class ChatGPTCodeGenerator:
    def __init__(self, api_key: str, model: str = "gpt-4"):
        """
        Initialize ChatGPT client for code generation
        
        Args:
            api_key: OpenAI API key
            model: GPT model to use
        """
        try:
            self.client = openai.OpenAI(api_key=api_key)
            self.model = model
            self.logger = logging.getLogger(__name__)
            self.logger.info(f"ChatGPT client initialized with model: {model}")
        except Exception as e:
            self.logger = logging.getLogger(__name__)
            self.logger.error(f"Failed to initialize ChatGPT client: {e}")
            raise
        
        # System prompt for ROS 2 code generation
        self.system_prompt = """
You are an expert ROS 2 programmer specializing in nova_carter3 robot control.
Generate Python code that uses rclpy and geometry_msgs to control the robot.

Available topics:
- /cmd_vel (geometry_msgs/Twist) - for robot movement
- /front_stereo_camera/left/image_raw (sensor_msgs/Image) - left camera
- /front_stereo_camera/right/image_raw (sensor_msgs/Image) - right camera

The robot should navigate to stand in front of red blocks detected in the scene.

Your response should contain executable Python code that:
1. Creates a ROS 2 node
2. Publishes to /cmd_vel for movement
3. Subscribes to camera topics if needed
4. Implements the requested behavior

Always include proper error handling and logging.
Respond with code only, no explanations.
"""
    
    def generate_robot_code(self, command: str, detection_info: Optional[Dict[str, Any]] = None) -> Optional[str]:
        """
        Generate ROS 2 code based on natural language command
        
        Args:
            command: Natural language command
            detection_info: Information about detected objects
            
        Returns:
            Generated Python code or None if failed
        """
        try:
            # Prepare user message with context
            user_message = f"Command: {command}\n"
            
            if detection_info:
                user_message += f"Object Detection Info: {json.dumps(detection_info)}\n"
            
            user_message += """
Generate ROS 2 Python code to control nova_carter3 robot based on this command.
The code should be a complete ROS 2 node that can be executed directly.
"""
            
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": user_message}
                ],
                temperature=0.3,
                max_tokens=2000,
                timeout=30  # 30 second timeout
            )
            
            if response.choices and response.choices[0].message.content:
                generated_code = response.choices[0].message.content
                self.logger.info(f"Generated code for command: {command}")
                return generated_code
            else:
                self.logger.error("Empty response from ChatGPT")
                return None
            
        except Exception as e:
            self.logger.error(f"Error generating code: {e}")
            return None
    
    def generate_navigation_code(self, target_position: Dict[str, float], 
                               current_position: Optional[Dict[str, float]] = None) -> Optional[str]:
        """
        Generate specific navigation code for moving to target position
        
        Args:
            target_position: Target position with x, y coordinates
            current_position: Current robot position (optional)
            
        Returns:
            Generated navigation code
        """
        command = f"Move the robot to position x={target_position['x']}, y={target_position['y']}"
        
        if current_position:
            command += f" from current position x={current_position['x']}, y={current_position['y']}"
        
        command += ". Use proportional control for smooth movement and stop when close to target."
        
        return self.generate_robot_code(command, {"target_position": target_position})
    
    def generate_block_approach_code(self, block_info: Dict[str, Any]) -> Optional[str]:
        """
        Generate code to approach detected red block
        
        Args:
            block_info: Information about detected red block
            
        Returns:
            Generated approach code
        """
        command = "Move the robot to stand in front of the detected red block. "
        command += "Use the camera feedback to center the block in view and approach it slowly. "
        command += "Stop at an appropriate distance (about 1 meter) in front of the block."
        
        return self.generate_robot_code(command, {"block_info": block_info})
    
    def validate_generated_code(self, code: str) -> bool:
        """
        Basic validation of generated code
        
        Args:
            code: Generated Python code
            
        Returns:
            True if code appears valid
        """
        required_imports = ["rclpy", "geometry_msgs"]
        required_patterns = ["/cmd_vel", "Twist"]
        
        for import_name in required_imports:
            if import_name not in code:
                self.logger.warning(f"Missing required import: {import_name}")
                return False
        
        for pattern in required_patterns:
            if pattern not in code:
                self.logger.warning(f"Missing required pattern: {pattern}")
                return False
        
        return True
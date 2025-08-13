#!/usr/bin/env python3
"""
Main Orchestration Script for Nova Carter3 Robot Control
Integrates speech recognition, object detection, ChatGPT, and robot control
"""

import rclpy
import cv2
import numpy as np
import logging
import threading
import time
import os
from typing import Optional, Dict, Any

from intelligent_move.src.speech_recognition_module import SpeechRecognitionModule
from intelligent_move.src.object_detection import RedBlockDetector
from intelligent_move.src.chatgpt_integration import ChatGPTCodeGenerator
from intelligent_move.src.robot_controller import NovaCarterController

class RobotOrchestrator:
    def __init__(self):
        """Initialize the robot orchestration system"""
        # Setup logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
        # Initialize ROS 2
        if not rclpy.ok():
            rclpy.init()
        
        # Initialize components
        self.speech_recognizer = SpeechRecognitionModule()
        self.object_detector = RedBlockDetector()
        self.robot_controller = NovaCarterController()
        
        # Initialize ChatGPT (requires API key)
        api_key = os.getenv('OPENAI_API_KEY')
        if not api_key:
            self.logger.warning("OPENAI_API_KEY environment variable not set - ChatGPT features disabled")
            self.code_generator = None
        else:
            self.code_generator = ChatGPTCodeGenerator(api_key)
        
        # State variables
        self.is_running = False
        self.current_task = None
        self.last_detection = None
        
        self.logger.info("Robot orchestrator initialized")
    
    def detect_red_block(self) -> Optional[Dict[str, Any]]:
        """Detect red block in current camera view"""
        image = self.robot_controller.get_current_image()
        
        if image is None:
            self.logger.warning("No camera image available")
            return None
        
        # Find closest red block
        red_block = self.object_detector.find_closest_red_block(image)
        
        if red_block:
            x1, y1, x2, y2, confidence = red_block
            center_x, center_y = self.object_detector.get_block_center(red_block)
            
            block_info = {
                'bbox': (x1, y1, x2, y2),
                'confidence': confidence,
                'center': (center_x, center_y),
                'image_width': image.shape[1],
                'image_height': image.shape[0]
            }
            
            self.logger.info(f"Red block detected at center ({center_x}, {center_y}) with confidence {confidence:.2f}")
            return block_info
        
        return None
    
    def process_voice_command(self, command: str):
        """Process voice command and execute robot actions"""
        self.logger.info(f"Processing command: {command}")
        
        # Detect current red block
        block_info = self.detect_red_block()
        
        if block_info:
            # Generate code to approach the red block
            if self.code_generator:
                generated_code = self.code_generator.generate_block_approach_code(block_info)
                
                if generated_code and self.code_generator.validate_generated_code(generated_code):
                    self.logger.info("Executing generated robot control code")
                    self.execute_robot_action(generated_code)
                else:
                    self.logger.error("Failed to generate valid robot control code")
                    self.fallback_approach_block(block_info)
            else:
                self.fallback_approach_block(block_info)
        else:
            # No red block detected, generate general movement code
            if self.code_generator:
                generated_code = self.code_generator.generate_robot_code(command)
                
                if generated_code and self.code_generator.validate_generated_code(generated_code):
                    self.logger.info("Executing generated robot control code")
                    self.execute_robot_action(generated_code)
                else:
                    self.logger.error("Failed to generate valid robot control code")
                    self.fallback_movement(command)
            else:
                self.fallback_movement(command)
    
    def execute_robot_action(self, code: str):
        """Execute generated robot control code"""
        try:
            # Execute in separate thread to avoid blocking
            execution_thread = threading.Thread(
                target=self.robot_controller.execute_generated_code,
                args=(code,)
            )
            execution_thread.daemon = True
            execution_thread.start()
            
        except Exception as e:
            self.logger.error(f"Error executing robot action: {e}")
    
    def fallback_approach_block(self, block_info: Dict[str, Any]):
        """Fallback method to approach red block when ChatGPT fails"""
        self.logger.info("Using fallback approach for red block")
        
        center_x, center_y = block_info['center']
        image_width = block_info['image_width']
        image_height = block_info['image_height']
        
        # Use built-in approach method
        approach_thread = threading.Thread(
            target=self.robot_controller.approach_object,
            args=(center_x, center_y, image_width, image_height, 1.0)
        )
        approach_thread.daemon = True
        approach_thread.start()
    
    def fallback_movement(self, command: str):
        """Fallback movement when ChatGPT fails"""
        self.logger.info("Using fallback movement commands")
        
        command_lower = command.lower()
        
        if "forward" in command_lower or "ahead" in command_lower:
            self.robot_controller.move_forward(1.0)
        elif "backward" in command_lower or "back" in command_lower:
            self.robot_controller.move_backward(1.0)
        elif "left" in command_lower:
            self.robot_controller.turn_left(45.0)
        elif "right" in command_lower:
            self.robot_controller.turn_right(45.0)
        elif "stop" in command_lower:
            self.robot_controller.stop_movement()
        else:
            self.logger.warning(f"Unknown command: {command}")
    
    def continuous_monitoring(self):
        """Continuously monitor for red blocks and update robot behavior"""
        while self.is_running:
            try:
                # Detect red block
                block_info = self.detect_red_block()
                
                if block_info:
                    self.last_detection = block_info
                    
                    # If robot is not currently moving, start approaching
                    if not self.robot_controller.is_moving:
                        self.logger.info("Red block detected, starting approach")
                        self.fallback_approach_block(block_info)
                
                # Spin ROS 2 callbacks
                rclpy.spin_once(self.robot_controller, timeout_sec=0.1)
                
                time.sleep(0.1)  # 10 Hz monitoring
                
            except Exception as e:
                self.logger.error(f"Error in continuous monitoring: {e}")
                time.sleep(1.0)
    
    def start_voice_control(self):
        """Start voice control loop"""
        self.logger.info("Starting voice control system")
        
        def voice_callback(command):
            self.process_voice_command(command)
        
        # Start continuous monitoring in separate thread
        monitoring_thread = threading.Thread(target=self.continuous_monitoring)
        monitoring_thread.daemon = True
        monitoring_thread.start()
        
        # Start voice recognition
        self.speech_recognizer.continuous_listen(voice_callback)
    
    def run_manual_mode(self):
        """Run system in manual command mode"""
        self.logger.info("Starting manual command mode")
        self.is_running = True
        
        # Start continuous monitoring
        monitoring_thread = threading.Thread(target=self.continuous_monitoring)
        monitoring_thread.daemon = True
        monitoring_thread.start()
        
        try:
            while self.is_running:
                command = input("Enter command (or 'quit' to exit): ")
                
                if command.lower() == 'quit':
                    break
                
                self.process_voice_command(command)
                
        except KeyboardInterrupt:
            self.logger.info("Manual mode interrupted")
        finally:
            self.shutdown()
    
    def run_auto_mode(self):
        """Run system in autonomous mode - find and approach red block"""
        self.logger.info("Starting autonomous red block detection mode")
        self.is_running = True
        
        # Start continuous monitoring
        monitoring_thread = threading.Thread(target=self.continuous_monitoring)
        monitoring_thread.daemon = True
        monitoring_thread.start()
        
        try:
            while self.is_running:
                # Check for red block detection
                block_info = self.detect_red_block()
                
                if block_info:
                    self.logger.info("Red block found, approaching...")
                    self.fallback_approach_block(block_info)
                    
                    # Wait for approach to complete
                    time.sleep(2.0)
                    
                    # Check if we're close enough
                    new_block_info = self.detect_red_block()
                    if new_block_info:
                        bbox = new_block_info['bbox']
                        block_area = (bbox[2] - bbox[0]) * (bbox[3] - bbox[1])
                        
                        # If block is large enough, we're close
                        if block_area > 50000:  # Threshold for "close enough"
                            self.logger.info("Successfully reached red block!")
                            self.robot_controller.stop_movement()
                            break
                else:
                    # No block detected, search by rotating
                    self.logger.info("No red block detected, searching...")
                    self.robot_controller.turn_right(30.0)
                    time.sleep(1.0)
                
                time.sleep(0.5)
                
        except KeyboardInterrupt:
            self.logger.info("Autonomous mode interrupted")
        finally:
            self.shutdown()
    
    def shutdown(self):
        """Shutdown the orchestrator"""
        self.logger.info("Shutting down robot orchestrator")
        self.is_running = False
        
        # Stop robot movement
        if hasattr(self, 'robot_controller') and self.robot_controller:
            self.robot_controller.stop_movement()
            self.robot_controller.shutdown()
        
        # Shutdown ROS 2 if it was initialized
        if rclpy.ok():
            rclpy.shutdown()

def main():
    """Main function"""
    try:
        orchestrator = RobotOrchestrator()
        
        # Choose mode
        print("Nova Carter3 Robot Control System")
        print("1. Voice Control Mode")
        print("2. Manual Command Mode")
        print("3. Autonomous Red Block Detection Mode")
        
        choice = input("Select mode (1-3): ")
        
        if choice == "1":
            orchestrator.start_voice_control()
        elif choice == "2":
            orchestrator.run_manual_mode()
        elif choice == "3":
            orchestrator.run_auto_mode()
        else:
            print("Invalid choice")
            
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'orchestrator' in locals():
            orchestrator.shutdown()

if __name__ == "__main__":
    main()
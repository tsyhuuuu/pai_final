#!/usr/bin/env python3
"""
YOLOv13 Object Detection Module for Red Block Detection
Detects red blocks in camera feed from nova_carter3
"""

import cv2
import numpy as np
import torch
from ultralytics import YOLO
import logging
from typing import List, Tuple, Optional

class RedBlockDetector:
    def __init__(self, model_path: str = "yolov8n.pt"):
        """
        Initialize YOLOv8 model for object detection
        Note: YOLOv13 is not officially released, using YOLOv8 as alternative
        """
        self.model = YOLO(model_path)
        self.logger = logging.getLogger(__name__)
        
    def detect_red_objects(self, image: np.ndarray) -> List[Tuple[int, int, int, int, float]]:
        """
        Detect objects in image and filter for red-colored items
        
        Args:
            image: Input image as numpy array
            
        Returns:
            List of bounding boxes (x1, y1, x2, y2, confidence) for red objects
        """
        results = self.model(image)
        red_objects = []
        
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    # Get bounding box coordinates
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    confidence = box.conf[0].cpu().numpy()
                    
                    # Extract region of interest
                    roi = image[int(y1):int(y2), int(x1):int(x2)]
                    
                    # Check if object is predominantly red
                    if self._is_red_object(roi):
                        red_objects.append((int(x1), int(y1), int(x2), int(y2), float(confidence)))
                        
        return red_objects
    
    def _is_red_object(self, roi: np.ndarray, red_threshold: float = 0.3) -> bool:
        """
        Check if region of interest contains predominantly red color
        
        Args:
            roi: Region of interest image
            red_threshold: Minimum ratio of red pixels
            
        Returns:
            True if object is predominantly red
        """
        if roi.size == 0:
            return False
            
        # Convert to HSV for better color filtering
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Define red color range in HSV
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        
        # Create masks for red color
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = mask1 + mask2
        
        # Calculate ratio of red pixels
        red_pixels = cv2.countNonZero(red_mask)
        total_pixels = roi.shape[0] * roi.shape[1]
        
        red_ratio = red_pixels / total_pixels if total_pixels > 0 else 0
        
        return red_ratio > red_threshold
    
    def find_closest_red_block(self, image: np.ndarray) -> Optional[Tuple[int, int, int, int, float]]:
        """
        Find the closest red block in the image
        
        Args:
            image: Input image
            
        Returns:
            Bounding box of closest red block or None if not found
        """
        red_objects = self.detect_red_objects(image)
        
        if not red_objects:
            return None
            
        # Find largest bounding box (assumed to be closest)
        largest_area = 0
        closest_block = None
        
        for obj in red_objects:
            x1, y1, x2, y2, conf = obj
            area = (x2 - x1) * (y2 - y1)
            
            if area > largest_area:
                largest_area = area
                closest_block = obj
                
        return closest_block
    
    def get_block_center(self, bbox: Tuple[int, int, int, int, float]) -> Tuple[int, int]:
        """
        Get center coordinates of bounding box
        
        Args:
            bbox: Bounding box (x1, y1, x2, y2, confidence)
            
        Returns:
            Center coordinates (x, y)
        """
        x1, y1, x2, y2, _ = bbox
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        return center_x, center_y
    
    def visualize_detection(self, image: np.ndarray, red_objects: List[Tuple[int, int, int, int, float]]) -> np.ndarray:
        """
        Draw bounding boxes on image for visualization
        
        Args:
            image: Input image
            red_objects: List of detected red objects
            
        Returns:
            Image with bounding boxes drawn
        """
        vis_image = image.copy()
        
        for obj in red_objects:
            x1, y1, x2, y2, conf = obj
            cv2.rectangle(vis_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(vis_image, f"Red Block: {conf:.2f}", 
                       (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
        return vis_image
"""
Advanced traffic sign detection for WRO 2025 competition
Detects red and green pillars with precise color matching and validation
"""

import cv2
import numpy as np
from typing import List, Dict, Optional, Tuple
from dataclasses import dataclass
from config.camera_config import VISION_CONFIG
from config.vehicle_config import RULES

@dataclass
class TrafficSign:
    """Detected traffic sign information"""
    color: str  # "red" or "green"
    position: Tuple[int, int]  # (x, y) center coordinates
    bounding_box: Tuple[int, int, int, int]  # (x, y, w, h)
    confidence: float
    distance_estimate: float  # Estimated distance in cm
    aspect_ratio: float
    area: int

class SignDetector:
    """
    Advanced traffic sign detector for WRO 2025 competition
    Uses color segmentation, shape analysis, and temporal tracking
    """
    
    def __init__(self):
        self.previous_signs = []  # For tracking and validation
        self.min_sign_area = 100  # Minimum pixel area for valid sign
        self.max_sign_area = 5000  # Maximum pixel area
        self.aspect_ratio_range = (0.5, 2.0)  # Width/height range for pillars
        
        # Color calibration for competition environment
        self.color_ranges = VISION_CONFIG.COLOR_RANGES
        
        # Sign tracking parameters
        self.tracking_history = []
        self.max_tracking_frames = 10
        
        print("✅ Sign detector initialized for WRO 2025 competition")
    
    def detect_signs(self, frame: np.ndarray) -> List[TrafficSign]:
        """
        Detect traffic signs in the frame using multi-stage validation
        Returns list of validated traffic signs
        """
        try:
            # Stage 1: Color-based segmentation
            red_signs = self._detect_color_signs(frame, 'red_sign', 'red')
            green_signs = self._detect_color_signs(frame, 'green_sign', 'green')
            
            all_signs = red_signs + green_signs
            
            # Stage 2: Shape validation and filtering
            validated_signs = self._validate_sign_shapes(all_signs)
            
            # Stage 3: Temporal tracking and validation
            tracked_signs = self._apply_temporal_tracking(validated_signs)
            
            return tracked_signs
            
        except Exception as e:
            print(f"❌ Sign detection error: {e}")
            return []
    
    def _detect_color_signs(self, frame: np.ndarray, color_range_name: str, color: str) -> List[TrafficSign]:
        """Detect signs of specific color"""
        signs = []
        
        # Convert to HSV for better color segmentation
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        
        # Create color mask
        if color_range_name not in self.color_ranges:
            return signs
            
        color_range = self.color_ranges[color_range_name]
        lower = np.array(color_range['lower'])
        upper = np.array(color_range['upper'])
        mask = cv2.inRange(hsv, lower, upper)
        
        # Apply morphological operations to clean up the mask
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            # Filter by area
            area = cv2.contourArea(contour)
            if area < self.min_sign_area or area > self.max_sign_area:
                continue
            
            # Get bounding rectangle
            x, y, w, h = cv2.boundingRect(contour)
            
            # Calculate aspect ratio (should be roughly square for pillars)
            aspect_ratio = w / h if h > 0 else 0
            
            if not (self.aspect_ratio_range[0] < aspect_ratio < self.aspect_ratio_range[1]):
                continue
            
            # Calculate confidence based on shape regularity
            hull = cv2.convexHull(contour)
            hull_area = cv2.contourArea(hull)
            solidity = area / hull_area if hull_area > 0 else 0
            
            # Additional shape validation using contour approximation
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # Pillars should have relatively simple shape (4-8 vertices)
            if len(approx) < 4 or len(approx) > 10:
                continue
            
            # Estimate distance based on bounding box height
            distance = self._estimate_distance_from_height(h)
            
            # Calculate position in normalized coordinates (-1 to +1)
            frame_center_x = frame.shape[1] // 2
            sign_center_x = x + w // 2
            normalized_x = (sign_center_x - frame_center_x) / frame_center_x
            
            # Create sign object
            sign = TrafficSign(
                color=color,
                position=(normalized_x, (y + h // 2) / frame.shape[0]),
                bounding_box=(x, y, w, h),
                confidence=min(1.0, solidity * (area / 1000)),
                distance_estimate=distance,
                aspect_ratio=aspect_ratio,
                area=area
            )
            
            signs.append(sign)
        
        return signs
    
    def _validate_sign_shapes(self, signs: List[TrafficSign]) -> List[TrafficSign]:
        """Validate signs based on shape characteristics"""
        validated_signs = []
        
        for sign in signs:
            # Additional validation based on competition rules
            # Signs are 50x50x100mm pillars, so they should appear as upright rectangles
            
            # Check if the sign is reasonably proportioned
            if sign.aspect_ratio < 0.3 or sign.aspect_ratio > 3.0:
                continue
            
            # Check if the sign is too close to the image edges (likely false positive)
            x_norm, y_norm = sign.position
            if abs(x_norm) > 0.9 or y_norm > 0.9:
                continue
            
            # Boost confidence for signs that match expected size at estimated distance
            expected_height = self._calculate_expected_height(sign.distance_estimate)
            actual_height = sign.bounding_box[3]
            height_ratio = actual_height / expected_height if expected_height > 0 else 0
            
            if 0.5 < height_ratio < 2.0:  # Within reasonable range
                sign.confidence *= 1.2
            
            validated_signs.append(sign)
        
        return validated_signs
    
    def _apply_temporal_tracking(self, current_signs: List[TrafficSign]) -> List[TrafficSign]:
        """Apply temporal tracking to validate signs across multiple frames"""
        if not current_signs:
            # No current signs, decay tracking history
            self.tracking_history = [signs for signs in self.tracking_history if len(signs) > 0]
            return []
        
        # Add current signs to tracking history
        self.tracking_history.append(current_signs)
        if len(self.tracking_history) > self.max_tracking_frames:
            self.tracking_history.pop(0)
        
        # Look for signs that appear consistently across frames
        reliable_signs = []
        
        for sign in current_signs:
            # Count how many times this sign (or similar) appears in history
            appearance_count = 0
            for historical_signs in self.tracking_history:
                if self._is_similar_sign(sign, historical_signs):
                    appearance_count += 1
            
            # Boost confidence for consistent signs
            consistency_ratio = appearance_count / len(self.tracking_history)
            if consistency_ratio > 0.5:  # Appears in more than 50% of recent frames
                sign.confidence *= (1.0 + consistency_ratio)
                reliable_signs.append(sign)
        
        return reliable_signs
    
    def _is_similar_sign(self, sign: TrafficSign, historical_signs: List[TrafficSign]) -> bool:
        """Check if a similar sign exists in historical data"""
        for historical_sign in historical_signs:
            if historical_sign.color != sign.color:
                continue
            
            # Check position similarity
            x1, y1 = sign.position
            x2, y2 = historical_sign.position
            position_distance = np.sqrt((x1 - x2)**2 + (y1 - y2)**2)
            
            if position_distance < 0.2:  # Similar position
                return True
        
        return False
    
    def _estimate_distance_from_height(self, pixel_height: int) -> float:
        """Estimate distance to sign based on its pixel height"""
        # Based on camera geometry and known sign height (100mm)
        # Simplified calculation: distance = (focal_length * real_height) / pixel_height
        
        # Approximate focal length for OV5647 at 320x240 resolution
        focal_length_pixels = 300  # Approximate value, should be calibrated
        
        real_height_mm = 100  # Sign height from competition rules
        distance_mm = (focal_length_pixels * real_height_mm) / pixel_height if pixel_height > 0 else 1000
        
        return distance_mm / 10.0  # Convert to cm
    
    def _calculate_expected_height(self, distance_cm: float) -> float:
        """Calculate expected pixel height for a sign at given distance"""
        if distance_cm <= 0:
            return 0
        
        focal_length_pixels = 300
        real_height_mm = 100
        expected_height = (focal_length_pixels * real_height_mm) / (distance_cm * 10)
        
        return expected_height
    
    def get_detection_statistics(self) -> Dict[str, any]:
        """Get sign detection performance statistics"""
        return {
            "tracking_history_length": len(self.tracking_history),
            "min_sign_area": self.min_sign_area,
            "max_sign_area": self.max_sign_area,
            "aspect_ratio_range": self.aspect_ratio_range
        }

# Global sign detector instance
sign_detector = SignDetector()
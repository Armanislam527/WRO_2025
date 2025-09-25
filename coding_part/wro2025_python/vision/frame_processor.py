"""
Main frame processor for WRO 2025 vision system
Coordinates lane detection, sign detection, and parking detection
Optimized for real-time performance on Pi Zero 2W
"""

import time
import cv2
import numpy as np
from typing import Dict, Tuple, Optional
from dataclasses import dataclass
from config.camera_config import VISION_CONFIG
from core.shared_memory import VisionData

@dataclass
class ProcessingResult:
    """Results from frame processing"""
    lane_center: float = 0.0
    lane_confidence: float = 0.0
    traffic_signs: list = None  # List of detected signs
    parking_spots: list = None  # List of detected parking spots
    processing_time: float = 0.0
    frame_timestamp: float = 0.0

class FrameProcessor:
    """
    Main frame processing coordinator
    Manages multiple detection algorithms and balances CPU load
    """
    
    def __init__(self):
        self.frame_count = 0
        self.processing_times = []
        self.enabled_detectors = {
            'lane_detection': True,
            'sign_detection': True, 
            'parking_detection': False  # Only enable after 3 laps
        }
        
        # Initialize OpenCV optimizations for Pi Zero 2W
        self._setup_opencv_optimizations()
        
        # Color ranges for competition elements (from config)
        self.color_ranges = VISION_CONFIG.COLOR_RANGES
        
        print("✅ Frame processor initialized with OpenCV optimizations")
    
    def process_frame(self, frame: np.ndarray) -> VisionData:
        """
        Process a single frame and extract competition-relevant information
        Returns VisionData structure for control system
        """
        start_time = time.time()
        self.frame_count += 1
        
        if frame is None or frame.size == 0:
            return VisionData()
        
        # Preprocess frame for efficient detection
        processed_frame = self._preprocess_frame(frame)
        
        # Initialize result structure
        result = ProcessingResult()
        result.frame_timestamp = start_time
        
        # Run enabled detectors
        if self.enabled_detectors['lane_detection']:
            lane_result = self._detect_lanes(processed_frame)
            result.lane_center = lane_result[0]
            result.lane_confidence = lane_result[1]
        
        if self.enabled_detectors['sign_detection']:
            result.traffic_signs = self._detect_traffic_signs(processed_frame)
        
        if self.enabled_detectors['parking_detection']:
            result.parking_spots = self._detect_parking_spots(processed_frame)
        
        # Calculate processing time
        result.processing_time = (time.time() - start_time) * 1000  # Convert to ms
        self.processing_times.append(result.processing_time)
        
        # Keep only recent processing times for average
        if len(self.processing_times) > 100:
            self.processing_times = self.processing_times[-100:]
        
        # Convert to VisionData for shared memory
        return self._convert_to_vision_data(result)
    
    def _preprocess_frame(self, frame: np.ndarray) -> np.ndarray:
        """Preprocess frame for efficient detection"""
        # Resize if needed (should already be resized by camera)
        if frame.shape[1] != 320 or frame.shape[0] != 240:
            frame = cv2.resize(frame, (320, 240))
        
        # Apply Gaussian blur for noise reduction
        blurred = cv2.GaussianBlur(frame, VISION_CONFIG.GAUSSIAN_BLUR, 0)
        
        # Convert to HSV for color detection
        hsv = cv2.cvtColor(blurred, cv2.COLOR_RGB2HSV)
        
        return hsv
    
    def _detect_lanes(self, hsv_frame: np.ndarray) -> Tuple[float, float]:
        """Detect lane markings and calculate center position"""
        try:
            # Create masks for orange and blue lines
            orange_mask = self._create_color_mask(hsv_frame, 'orange_line')
            blue_mask = self._create_color_mask(hsv_frame, 'blue_line')
            
            # Combine masks for lane detection
            lane_mask = cv2.bitwise_or(orange_mask, blue_mask)
            
            # Apply morphological operations to clean up the mask
            kernel = np.ones((3, 3), np.uint8)
            lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_CLOSE, kernel)
            
            # Find contours in the mask
            contours, _ = cv2.findContours(lane_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if not contours:
                return 0.0, 0.0  # Centered, no confidence
            
            # Find the largest contour (assumed to be the lane)
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Calculate moments to find center of mass
            M = cv2.moments(largest_contour)
            if M["m00"] == 0:
                return 0.0, 0.0
            
            # Calculate center x-coordinate relative to frame center
            frame_center_x = hsv_frame.shape[1] // 2
            contour_center_x = int(M["m10"] / M["m00"])
            
            # Normalize to -1.0 (left) to +1.0 (right)
            lane_center = (contour_center_x - frame_center_x) / frame_center_x
            
            # Calculate confidence based on contour area and solidity
            area = cv2.contourArea(largest_contour)
            hull = cv2.convexHull(largest_contour)
            hull_area = cv2.contourArea(hull)
            solidity = area / hull_area if hull_area > 0 else 0
            
            confidence = min(1.0, (area / 1000) * solidity)  # Normalize confidence
            
            return lane_center, confidence
            
        except Exception as e:
            print(f"❌ Lane detection error: {e}")
            return 0.0, 0.0
    
    def _detect_traffic_signs(self, hsv_frame: np.ndarray) -> list:
        """Detect red and green traffic signs (pillars)"""
        detected_signs = []
        
        try:
            # Detect red signs
            red_mask = self._create_color_mask(hsv_frame, 'red_sign')
            red_signs = self._find_sign_contours(red_mask, 'red')
            detected_signs.extend(red_signs)
            
            # Detect green signs  
            green_mask = self._create_color_mask(hsv_frame, 'green_sign')
            green_signs = self._find_sign_contours(green_mask, 'green')
            detected_signs.extend(green_signs)
            
        except Exception as e:
            print(f"❌ Sign detection error: {e}")
        
        return detected_signs
    
    def _detect_parking_spots(self, hsv_frame: np.ndarray) -> list:
        """Detect magenta parking spot markers"""
        parking_spots = []
        
        try:
            # Detect magenta parking markers
            magenta_mask = self._create_color_mask(hsv_frame, 'magenta_parking')
            
            # Find contours of parking markers
            contours, _ = cv2.findContours(magenta_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < 100:  # Minimum area threshold
                    continue
                
                # Approximate the contour to simplify shape
                epsilon = 0.02 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                
                # Parking markers should be rectangular
                if len(approx) == 4:
                    # Calculate center and width
                    x, y, w, h = cv2.boundingRect(contour)
                    center_x = x + w // 2
                    normalized_center = (center_x - 160) / 160  # -1 to +1
                    
                    parking_spots.append({
                        'center': normalized_center,
                        'width': w / 320.0,  # Normalized width
                        'confidence': min(1.0, area / 1000)
                    })
                    
        except Exception as e:
            print(f"❌ Parking detection error: {e}")
        
        return parking_spots
    
    def _create_color_mask(self, hsv_frame: np.ndarray, color_name: str) -> np.ndarray:
        """Create a mask for specific color range"""
        if color_name not in self.color_ranges:
            return np.zeros(hsv_frame.shape[:2], dtype=np.uint8)
        
        color_range = self.color_ranges[color_name]
        lower = np.array(color_range['lower'])
        upper = np.array(color_range['upper'])
        
        return cv2.inRange(hsv_frame, lower, upper)
    
    def _find_sign_contours(self, mask: np.ndarray, color: str) -> list:
        """Find sign contours in the mask and return sign information"""
        signs = []
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 50:  # Minimum area for signs
                continue
            
            # Calculate position and size
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = w / h if h > 0 else 0
            
            # Signs should be roughly square (pillars)
            if 0.5 < aspect_ratio < 2.0:
                center_x = x + w // 2
                normalized_x = (center_x - 160) / 160  # -1 to +1
                
                signs.append({
                    'color': color,
                    'position': normalized_x,
                    'confidence': min(1.0, area / 500),  # Normalize confidence
                    'distance_estimate': self._estimate_distance(h)  # Based on height in frame
                })
        
        return signs
    
    def _estimate_distance(self, pixel_height: int) -> float:
        """Estimate distance to object based on pixel height (simplified)"""
        # This is a simplified distance estimation
        # In real implementation, would use camera calibration
        if pixel_height <= 0:
            return 1000  # Far away
        
        # Approximate: object appears smaller as distance increases
        return (100.0 / pixel_height) * 50  # Rough distance in cm
    
    def _convert_to_vision_data(self, result: ProcessingResult) -> VisionData:
        """Convert processing results to VisionData structure"""
        vision_data = VisionData()
        vision_data.lane_center = result.lane_center
        vision_data.lane_confidence = result.lane_confidence
        vision_data.frame_timestamp = result.frame_timestamp
        
        # Process traffic signs
        if result.traffic_signs:
            # Use the most confident sign
            best_sign = max(result.traffic_signs, key=lambda x: x['confidence'], default=None)
            if best_sign and best_sign['confidence'] > 0.3:
                vision_data.traffic_sign_detected = True
                vision_data.sign_color = best_sign['color']
                vision_data.sign_position = best_sign['position']
        
        # Process parking spots
        if result.parking_spots:
            best_spot = max(result.parking_spots, key=lambda x: x['confidence'], default=None)
            if best_spot and best_spot['confidence'] > 0.4:
                vision_data.parking_detected = True
                vision_data.parking_center = best_spot['center']
                vision_data.parking_width = best_spot['width']
        
        return vision_data
    
    def _setup_opencv_optimizations(self):
        """Setup OpenCV for optimal performance on Pi Zero 2W"""
        # Use single-threaded mode to avoid context switching overhead
        cv2.setNumThreads(0)
        
        # Precompile often-used kernels
        self.morph_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    
    def enable_parking_detection(self, enable: bool = True):
        """Enable or disable parking detection"""
        self.enabled_detectors['parking_detection'] = enable
        print(f"{'🅿️ Enabled' if enable else '❌ Disabled'} parking detection")
    
    def get_performance_stats(self) -> Dict[str, float]:
        """Get frame processing performance statistics"""
        if not self.processing_times:
            return {"average_time": 0, "frame_count": 0}
        
        avg_time = sum(self.processing_times) / len(self.processing_times)
        return {
            "average_time": avg_time,
            "frame_count": self.frame_count,
            "fps": 1000 / avg_time if avg_time > 0 else 0
        }

# Global frame processor instance
frame_processor = FrameProcessor()
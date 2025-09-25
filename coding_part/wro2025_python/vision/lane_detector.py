"""
Advanced lane detection algorithms for WRO 2025 competition
Uses multiple techniques for robust lane detection in various conditions
"""

import cv2
import numpy as np
from typing import Tuple, List, Optional

class LaneDetector:
    """
    Advanced lane detector using edge detection, Hough transforms, and curve fitting
    Optimized for WRO 2025 track with orange and blue lane markings
    """
    
    def __init__(self):
        self.previous_lanes = []  # For temporal smoothing
        self.smooth_factor = 0.7  # Weight for previous detections
        
        # Lane detection parameters
        self.canny_threshold1 = 50
        self.canny_threshold2 = 150
        self.hough_threshold = 20
        self.min_line_length = 30
        self.max_line_gap = 20
        
    def detect_lanes(self, frame: np.ndarray) -> Tuple[float, float]:
        """
        Detect lanes in the frame and return center position and confidence
        Uses a multi-stage approach for robust detection
        """
        try:
            # Stage 1: Preprocessing
            processed = self._preprocess_frame(frame)
            
            # Stage 2: Edge detection
            edges = self._detect_edges(processed)
            
            # Stage 3: Lane line detection
            lane_lines = self._detect_lane_lines(edges)
            
            # Stage 4: Calculate lane center
            center, confidence = self._calculate_lane_center(lane_lines, frame.shape[1])
            
            # Stage 5: Temporal smoothing
            center, confidence = self._apply_temporal_smoothing(center, confidence)
            
            return center, confidence
            
        except Exception as e:
            print(f"❌ Lane detection error: {e}")
            return 0.0, 0.0
    
    def _preprocess_frame(self, frame: np.ndarray) -> np.ndarray:
        """Preprocess frame for lane detection"""
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        
        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Enhance contrast using CLAHE (Contrast Limited Adaptive Histogram Equalization)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(blurred)
        
        return enhanced
    
    def _detect_edges(self, image: np.ndarray) -> np.ndarray:
        """Detect edges using Canny algorithm with adaptive thresholds"""
        # Calculate adaptive thresholds based on image statistics
        mean_intensity = np.mean(image)
        self.canny_threshold1 = max(30, int(mean_intensity * 0.5))
        self.canny_threshold2 = max(90, int(mean_intensity * 1.5))
        
        return cv2.Canny(image, self.canny_threshold1, self.canny_threshold2)
    
    def _detect_lane_lines(self, edges: np.ndarray) -> List[Tuple[float, float]]:
        """Detect lane lines using Hough transform"""
        # Create region of interest (lower half of image where lanes are)
        height, width = edges.shape
        mask = np.zeros_like(edges)
        polygon = np.array([[
            (0, height),
            (width, height),
            (width, height // 2),
            (0, height // 2)
        ]], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        masked_edges = cv2.bitwise_and(edges, mask)
        
        # Detect lines using Hough transform
        lines = cv2.HoughLinesP(
            masked_edges,
            rho=1,
            theta=np.pi / 180,
            threshold=self.hough_threshold,
            minLineLength=self.min_line_length,
            maxLineGap=self.max_line_gap
        )
        
        if lines is None:
            return []
        
        # Convert lines to slope-intercept form
        lane_lines = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x2 - x1 == 0:  # Avoid division by zero
                continue
            
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - slope * x1
            
            # Filter out non-lane lines (based on slope)
            if 0.2 < abs(slope) < 2.0:  # Reasonable slope for lanes
                lane_lines.append((slope, intercept))
        
        return lane_lines
    
    def _calculate_lane_center(self, lane_lines: List[Tuple[float, float]], frame_width: int) -> Tuple[float, float]:
        """Calculate lane center from detected lines"""
        if not lane_lines:
            return 0.0, 0.0
        
        # Separate left and right lanes based on slope
        left_lines = [line for line in lane_lines if line[0] < 0]
        right_lines = [line for line in lane_lines if line[0] > 0]
        
        # Calculate average lines
        left_avg = self._average_lines(left_lines) if left_lines else None
        right_avg = self._average_lines(right_lines) if right_lines else None
        
        # Calculate lane center
        if left_avg and right_avg:
            # Both lanes detected - calculate center between them
            left_x = self._get_x_at_y(left_avg, frame_width // 2)  # At middle of frame
            right_x = self._get_x_at_y(right_avg, frame_width // 2)
            
            if left_x < right_x:  # Sanity check
                lane_center = (left_x + right_x) / 2
                frame_center = frame_width / 2
                normalized_center = (lane_center - frame_center) / frame_center
                confidence = 0.8
            else:
                normalized_center, confidence = 0.0, 0.3
                
        elif left_avg or right_avg:
            # Only one lane detected - estimate center
            detected_line = left_avg if left_avg else right_avg
            lane_width_estimate = frame_width * 0.6  # Estimated lane width
            
            if left_avg:
                # Left lane detected, center should be to the right
                left_x = self._get_x_at_y(left_avg, frame_width // 2)
                lane_center = left_x + lane_width_estimate / 2
            else:
                # Right lane detected, center should be to the left
                right_x = self._get_x_at_y(right_avg, frame_width // 2)
                lane_center = right_x - lane_width_estimate / 2
            
            frame_center = frame_width / 2
            normalized_center = (lane_center - frame_center) / frame_center
            confidence = 0.5
            
        else:
            normalized_center, confidence = 0.0, 0.1
        
        return normalized_center, confidence
    
    def _average_lines(self, lines: List[Tuple[float, float]]) -> Optional[Tuple[float, float]]:
        """Average multiple lines into a single representative line"""
        if not lines:
            return None
        
        avg_slope = np.mean([line[0] for line in lines])
        avg_intercept = np.mean([line[1] for line in lines])
        
        return (avg_slope, avg_intercept)
    
    def _get_x_at_y(self, line: Tuple[float, float], y: int) -> float:
        """Get x-coordinate for a given y-coordinate on the line"""
        slope, intercept = line
        return (y - intercept) / slope if slope != 0 else 0
    
    def _apply_temporal_smoothing(self, center: float, confidence: float) -> Tuple[float, float]:
        """Apply temporal smoothing to reduce jitter"""
        if not self.previous_lanes:
            self.previous_lanes.append((center, confidence))
            return center, confidence
        
        # Weighted average with previous detections
        prev_center, prev_confidence = self.previous_lanes[-1]
        
        if confidence > 0.5:  # Only smooth if current detection is confident
            smoothed_center = (self.smooth_factor * prev_center + 
                             (1 - self.smooth_factor) * center)
            smoothed_confidence = (self.smooth_factor * prev_confidence + 
                                 (1 - self.smooth_factor) * confidence)
        else:
            # Low confidence - rely more on previous detection
            smoothed_center = prev_center
            smoothed_confidence = prev_confidence * 0.8  # Decay confidence
        
        # Keep only recent history
        self.previous_lanes.append((smoothed_center, smoothed_confidence))
        if len(self.previous_lanes) > 10:
            self.previous_lanes.pop(0)
        
        return smoothed_center, smoothed_confidence

# Global lane detector instance
lane_detector = LaneDetector()
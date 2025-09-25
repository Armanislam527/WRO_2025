"""
Parking spot detection for WRO 2025 Obstacle Challenge
Detects magenta parking markers and calculates parking space geometry
"""

import cv2
import numpy as np
from typing import List, Dict, Optional, Tuple
from dataclasses import dataclass
from config.camera_config import VISION_CONFIG
from config.vehicle_config import RULES, VEHICLE

@dataclass
class ParkingSpot:
    """Detected parking spot information"""
    center_position: Tuple[float, float]  # Normalized coordinates (-1 to +1)
    width: float  # Normalized width (0 to 1)
    depth: float  # Normalized depth
    confidence: float
    bounding_box: Tuple[int, int, int, int]  # (x, y, w, h)
    markers_detected: int  # Number of markers detected (1 or 2)
    orientation: str  # "horizontal" or "vertical"

class ParkingDetector:
    """
    Advanced parking spot detector for WRO 2025 competition
    Detects magenta parking markers and calculates parking space parameters
    """
    
    def __init__(self):
        self.min_marker_area = 200
        self.max_marker_area = 3000
        self.parking_width_range = (0.3, 1.0)  # Normalized width range
        self.marker_aspect_ratio = (3.0, 10.0)  # Parking markers are rectangular
        
        # Parking space dimensions from competition rules
        self.real_parking_width = RULES.PARKING_LOT_WIDTH_MM / 1000.0  # Convert to meters
        self.real_parking_length = 1.5 * VEHICLE.WHEELBASE_MM / 1000.0  # 1.5x vehicle length
        
        # Detection parameters
        self.min_confidence = 0.4
        self.tracking_history = []
        
        print("✅ Parking detector initialized for WRO 2025 competition")
    
    def detect_parking_spots(self, frame: np.ndarray) -> List[ParkingSpot]:
        """
        Detect parking spots based on magenta markers
        Returns list of validated parking spots
        """
        try:
            # Stage 1: Detect parking markers
            markers = self._detect_parking_markers(frame)
            
            # Stage 2: Group markers into parking spots
            parking_spots = self._group_markers_into_spots(markers, frame.shape)
            
            # Stage 3: Validate parking spots
            validated_spots = self._validate_parking_spots(parking_spots)
            
            return validated_spots
            
        except Exception as e:
            print(f"❌ Parking detection error: {e}")
            return []
    
    def _detect_parking_markers(self, frame: np.ndarray) -> List[Dict]:
        """Detect individual parking markers (magenta rectangles)"""
        markers = []
        
        # Convert to HSV for color detection
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        
        # Create magenta mask
        if 'magenta_parking' not in VISION_CONFIG.COLOR_RANGES:
            return markers
            
        color_range = VISION_CONFIG.COLOR_RANGES['magenta_parking']
        lower = np.array(color_range['lower'])
        upper = np.array(color_range['upper'])
        mask = cv2.inRange(hsv, lower, upper)
        
        # Apply morphological operations
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_marker_area or area > self.max_marker_area:
                continue
            
            # Get bounding rectangle
            x, y, w, h = cv2.boundingRect(contour)
            
            # Check aspect ratio (parking markers are long rectangles)
            aspect_ratio = w / h if h > 0 else 0
            if aspect_ratio < 1:
                aspect_ratio = 1 / aspect_ratio  # Always >= 1
            
            if not (self.marker_aspect_ratio[0] < aspect_ratio < self.marker_aspect_ratio[1]):
                continue
            
            # Calculate marker properties
            center_x = x + w // 2
            center_y = y + h // 2
            
            # Normalize coordinates
            norm_center_x = (center_x - frame.shape[1] // 2) / (frame.shape[1] // 2)
            norm_center_y = center_y / frame.shape[0]
            
            # Determine orientation
            orientation = "horizontal" if w > h else "vertical"
            
            marker = {
                'center': (norm_center_x, norm_center_y),
                'bounding_box': (x, y, w, h),
                'aspect_ratio': aspect_ratio,
                'area': area,
                'orientation': orientation,
                'confidence': min(1.0, area / 2000)  # Normalize confidence
            }
            
            markers.append(marker)
        
        return markers
    
    def _group_markers_into_spots(self, markers: List[Dict], frame_shape: Tuple[int, int]) -> List[ParkingSpot]:
        """Group detected markers into complete parking spots"""
        spots = []
        
        if len(markers) < 1:
            return spots
        
        # Try to find pairs of markers that form a parking spot
        for i, marker1 in enumerate(markers):
            for j, marker2 in enumerate(markers[i+1:], i+1):
                if self._are_markers_compatible(marker1, marker2):
                    spot = self._create_parking_spot(marker1, marker2, frame_shape)
                    if spot:
                        spots.append(spot)
        
        # Also consider single markers as potential parking spots
        for marker in markers:
            spot = self._create_single_marker_spot(marker, frame_shape)
            if spot:
                spots.append(spot)
        
        return spots
    
    def _are_markers_compatible(self, marker1: Dict, marker2: Dict) -> bool:
        """Check if two markers can form a parking spot"""
        # Markers should have similar orientation
        if marker1['orientation'] != marker2['orientation']:
            return False
        
        # Markers should be reasonably close
        x1, y1 = marker1['center']
        x2, y2 = marker2['center']
        distance = np.sqrt((x1 - x2)**2 + (y1 - y2)**2)
        
        if distance > 0.8:  # Too far apart
            return False
        
        # Markers should have similar size
        size_ratio = marker1['area'] / marker2['area'] if marker2['area'] > 0 else 0
        if size_ratio < 0.5 or size_ratio > 2.0:
            return False
        
        return True
    
    def _create_parking_spot(self, marker1: Dict, marker2: Dict, frame_shape: Tuple[int, int]) -> Optional[ParkingSpot]:
        """Create a parking spot from two detected markers"""
        x1, y1 = marker1['center']
        x2, y2 = marker2['center']
        
        # Calculate spot center
        spot_center_x = (x1 + x2) / 2
        spot_center_y = (y1 + y2) / 2
        
        # Calculate spot width (distance between markers)
        marker_distance = np.sqrt((x1 - x2)**2 + (y1 - y2)**2)
        spot_width = marker_distance
        
        # Estimate depth based on marker size
        avg_area = (marker1['area'] + marker2['area']) / 2
        spot_depth = self._estimate_depth_from_area(avg_area)
        
        # Calculate combined bounding box
        x1_px, y1_px, w1, h1 = marker1['bounding_box']
        x2_px, y2_px, w2, h2 = marker2['bounding_box']
        
        min_x = min(x1_px, x2_px)
        min_y = min(y1_px, y2_px)
        max_x = max(x1_px + w1, x2_px + w2)
        max_y = max(y1_px + h1, y2_px + h2)
        
        bounding_box = (min_x, min_y, max_x - min_x, max_y - min_y)
        
        # Calculate confidence
        confidence = (marker1['confidence'] + marker2['confidence']) / 2
        
        # Determine orientation
        orientation = marker1['orientation']
        
        return ParkingSpot(
            center_position=(spot_center_x, spot_center_y),
            width=spot_width,
            depth=spot_depth,
            confidence=confidence,
            bounding_box=bounding_box,
            markers_detected=2,
            orientation=orientation
        )
    
    def _create_single_marker_spot(self, marker: Dict, frame_shape: Tuple[int, int]) -> Optional[ParkingSpot]:
        """Create a parking spot from a single detected marker"""
        # For single markers, estimate the parking spot based on typical dimensions
        x, y = marker['center']
        
        # Estimate width based on marker orientation and size
        if marker['orientation'] == 'horizontal':
            spot_width = 0.6  # Default width
            spot_depth = 0.3  # Default depth
        else:
            spot_width = 0.3  # Default width
            spot_depth = 0.6  # Default depth
        
        # Adjust based on marker size
        size_factor = marker['area'] / 1000  # Normalize
        spot_width *= (0.5 + size_factor)
        spot_depth *= (0.5 + size_factor)
        
        # Lower confidence for single markers
        confidence = marker['confidence'] * 0.7
        
        return ParkingSpot(
            center_position=(x, y),
            width=spot_width,
            depth=spot_depth,
            confidence=confidence,
            bounding_box=marker['bounding_box'],
            markers_detected=1,
            orientation=marker['orientation']
        )
    
    def _validate_parking_spots(self, spots: List[ParkingSpot]) -> List[ParkingSpot]:
        """Validate detected parking spots based on competition rules"""
        validated_spots = []
        
        for spot in spots:
            # Check if spot meets minimum confidence
            if spot.confidence < self.min_confidence:
                continue
            
            # Check if spot dimensions are reasonable
            if not (self.parking_width_range[0] < spot.width < self.parking_width_range[1]):
                continue
            
            # Check if spot is not too close to image edges
            x, y = spot.center_position
            if abs(x) > 0.9 or y > 0.9:
                continue
            
            validated_spots.append(spot)
        
        # Sort by confidence
        validated_spots.sort(key=lambda x: x.confidence, reverse=True)
        
        return validated_spots
    
    def _estimate_depth_from_area(self, area: float) -> float:
        """Estimate parking spot depth from marker area"""
        # Simplified estimation - would need camera calibration for accuracy
        if area < 500:
            return 0.3
        elif area < 1500:
            return 0.5
        else:
            return 0.7
    
    def get_detection_statistics(self) -> Dict[str, any]:
        """Get parking detection performance statistics"""
        return {
            "min_marker_area": self.min_marker_area,
            "max_marker_area": self.max_marker_area,
            "parking_width_range": self.parking_width_range,
            "min_confidence": self.min_confidence,
            "real_parking_width": self.real_parking_width,
            "real_parking_length": self.real_parking_length
        }

# Global parking detector instance
parking_detector = ParkingDetector()
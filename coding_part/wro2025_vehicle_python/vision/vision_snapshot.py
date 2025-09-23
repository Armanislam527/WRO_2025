# vision/vision_snapshot.py

"""
Defines the VisionSnapshot data structure, an advanced version for robust processing.
This structure holds detailed results of image processing for a single frame,
providing richer information for navigation decisions.
"""

from dataclasses import dataclass, field
from typing import Optional, Tuple, List
import time

@dataclass
class DetectedSign:
    """
    Represents a detected traffic sign with detailed information.
    """
    # Type of sign detected
    class SignType:
        LEFT = "LEFT"
        RIGHT = "RIGHT"
        UNKNOWN = "UNKNOWN"

    sign_type: str = SignType.UNKNOWN  # Use SignType.LEFT or SignType.RIGHT
    # Bounding box (x, y, width, height) in image coordinates
    bbox: Tuple[int, int, int, int] = field(default_factory=lambda: (0, 0, 0, 0))
    # Confidence level of the detection (0.0 to 1.0)
    confidence: float = 0.0
    # Centroid (x, y) of the detection
    centroid: Tuple[int, int] = field(default_factory=lambda: (0, 0))
    # Area of the bounding box (width * height)
    area: int = 0

    def is_valid(self) -> bool:
        """Check if the sign detection is considered valid (non-zero area, reasonable confidence)."""
        return self.area > 0 and self.confidence > 0.1 # Threshold can be adjusted

    def __str__(self):
        return (f"DetectedSign(Type: {self.sign_type}, Conf: {self.confidence:.2f}, "
                f"BBox: {self.bbox}, Centroid: {self.centroid})")

@dataclass
class DetectedEdgeLine:
    """
    Represents a detected line segment, typically from edge detection.
    Using the format from cv2.HoughLinesP: [x1, y1, x2, y2]
    """
    # Endpoints of the line segment
    line: Tuple[int, int, int, int] = field(default_factory=lambda: (0, 0, 0, 0))
    # Length of the line segment
    length: float = 0.0
    # Angle of the line segment in radians (relative to horizontal)
    angle: float = 0.0
    # Confidence or strength of the line (e.g., inverse of Hough accumulator value, or based on length)
    strength: float = 0.0 

    def is_valid(self) -> bool:
        """Check if the line detection is considered valid."""
        return self.length > 5.0 and self.strength > 0.0 # Thresholds can be adjusted

    def __str__(self):
        return (f"DetectedEdgeLine(Line: {self.line}, Len: {self.length:.1f}, "
                f"Angle: {self.angle:.2f}rad, Strength: {self.strength:.2f})")

@dataclass
class VisionSnapshot:
    """
    Advanced container for vision processing results for a single frame.
    Provides detailed information for robust navigation decisions.
    """
    # --- Timestamp ---
    timestamp: float = field(default_factory=time.time) # Time of processing

    # --- Traffic Sign Detection ---
    # Lists to hold potentially multiple detections
    left_signs: List[DetectedSign] = field(default_factory=list)
    right_signs: List[DetectedSign] = field(default_factory=list)
    
    # Convenience properties for the most likely sign (highest confidence)
    @property
    def primary_left_sign(self) -> Optional[DetectedSign]:
        """Get the left sign with the highest confidence, if any."""
        if not self.left_signs:
            return None
        # Filter valid signs first, then find the one with max confidence
        valid_signs = [s for s in self.left_signs if s.is_valid()]
        if not valid_signs:
             return None
        return max(valid_signs, key=lambda s: s.confidence)

    @property
    def primary_right_sign(self) -> Optional[DetectedSign]:
        """Get the right sign with the highest confidence, if any."""
        if not self.right_signs:
            return None
        valid_signs = [s for s in self.right_signs if s.is_valid()]
        if not valid_signs:
             return None
        return max(valid_signs, key=lambda s: s.confidence)

    # --- Track Edge Detection ---
    # Lists of detected line segments for left and right edges
    left_edge_lines: List[DetectedEdgeLine] = field(default_factory=list)
    right_edge_lines: List[DetectedEdgeLine] = field(default_factory=list)

    # Aggregated edge information (can be derived from lines or direct pixel analysis)
    # Average X position of detected left/right edges within the ROI (-1.0 indicates not found reliably)
    avg_left_edge_x: float = -1.0
    avg_right_edge_x: float = -1.0
    # Standard deviation or spread of edge positions (indicates confidence/track width)
    left_edge_std: float = 0.0
    right_edge_std: float = 0.0

    # --- Frame Metadata ---
    # Identifier for the frame (e.g., sequence number from camera)
    frame_id: int = 0
    # Processing time taken for this frame (ms)
    processing_time_ms: float = 0.0
    # ROI used for processing (if dynamic)
    # processing_roi: Optional[Tuple[int, int, int, int]] = None # (x, y, w, h)

    # --- Processing Flags ---
    # Indicates if signs were specifically searched for in this frame
    signs_searched: bool = False
    # Indicates if edges were specifically searched for in this frame
    edges_searched: bool = False

    def __str__(self):
        left_desc = f"{len(self.left_signs)} signs"
        right_desc = f"{len(self.right_signs)} signs"
        left_edge_desc = f"{len(self.left_edge_lines)} lines"
        right_edge_desc = f"{len(self.right_edge_lines)} lines"
        if self.primary_left_sign:
            left_desc += f" (Primary: {self.primary_left_sign.sign_type}@{self.primary_left_sign.centroid})"
        if self.primary_right_sign:
            right_desc += f" (Primary: {self.primary_right_sign.sign_type}@{self.primary_right_sign.centroid})"
        
        return (f"VisionSnapshot(TS: {self.timestamp:.3f}, FID: {self.frame_id}, "
                f"LSigns: {left_desc}, RSigns: {right_desc}, "
                f"LEdges: {left_edge_desc} (X_avg={self.avg_left_edge_x:.1f}), "
                f"REdges: {right_edge_desc} (X_avg={self.avg_right_edge_x:.1f}))")

    def reset(self):
        """Reset all fields to default/empty values."""
        self.timestamp = time.time()
        self.left_signs.clear()
        self.right_signs.clear()
        self.left_edge_lines.clear()
        self.right_edge_lines.clear()
        self.avg_left_edge_x = -1.0
        self.avg_right_edge_x = -1.0
        self.left_edge_std = 0.0
        self.right_edge_std = 0.0
        self.frame_id = 0
        self.processing_time_ms = 0.0
        # self.processing_roi = None
        self.signs_searched = False
        self.edges_searched = False

    def has_sign_conflict(self) -> bool:
        """
        Check if there's a potential conflict (both left and right signs detected with high confidence).
        This requires robust confidence scoring in the detection logic.
        """
        primary_left = self.primary_left_sign
        primary_right = self.primary_right_sign
        if primary_left and primary_right:
            # Both primary signs exist and are valid
            # Conflict if both are highly confident and reasonably close
            # This logic can be refined
            return (primary_left.confidence > 0.7 and primary_right.confidence > 0.7)
        return False

    def get_track_width_estimate(self) -> float:
        """
        Estimate the track width based on detected edges.
        Returns -1.0 if estimation is not possible.
        """
        if self.avg_left_edge_x >= 0 and self.avg_right_edge_x >= 0:
            return self.avg_right_edge_x - self.avg_left_edge_x
        # Could also try to estimate from lines if averages are not available
        # ...
        return -1.0

# Example usage (if run as script)
if __name__ == "__main__":
    # Default snapshot
    snap1 = VisionSnapshot()
    print("Default:", snap1)

    # Snapshot with detailed data
    left_sign = DetectedSign(
        sign_type=DetectedSign.SignType.LEFT,
        bbox=(50, 100, 30, 50),
        confidence=0.95,
        centroid=(65, 125),
        area=1500
    )
    right_sign = DetectedSign(
        sign_type=DetectedSign.SignType.RIGHT,
        bbox=(500, 110, 25, 45),
        confidence=0.88,
        centroid=(512, 132),
        area=1125
    )
    edge_line_left = DetectedEdgeLine(
        line=(10, 50, 15, 200),
        length=155.0,
        angle=1.5, # ~90 degrees
        strength=0.9
    )
    edge_line_right = DetectedEdgeLine(
        line=(600, 40, 610, 210),
        length=175.0,
        angle=1.4, # ~80 degrees
        strength=0.85
    )

    snap2 = VisionSnapshot(
        frame_id=100,
        left_signs=[left_sign],
        right_signs=[right_sign],
        left_edge_lines=[edge_line_left],
        right_edge_lines=[edge_line_right],
        avg_left_edge_x=12.5,
        avg_right_edge_x=605.0,
        left_edge_std=2.0,
        right_edge_std=1.5,
        signs_searched=True,
        edges_searched=True
    )
    print("\nWith Data:")
    print(snap2)
    print(f"Primary Left Sign: {snap2.primary_left_sign}")
    print(f"Primary Right Sign: {snap2.primary_right_sign}")
    print(f"Has Conflict? {snap2.has_sign_conflict()}")
    print(f"Estimated Track Width: {snap2.get_track_width_estimate():.1f}px")

    # Resetting
    snap2.reset()
    print("\nReset:")
    print(snap2)
    print(f"Primary Left Sign: {snap2.primary_left_sign}")

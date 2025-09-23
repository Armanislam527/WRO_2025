# vision/edge_detector.py
"""Detects track edges using Canny edge detection and HoughLinesP."""

import cv2
import numpy as np
import logging
from typing import List, Tuple

logger = logging.getLogger(__name__)

class EdgeDetector:
    """
    Detects track edges using Canny edge detection and the Hough transform.
    """

    def __init__(self):
        # --- Canny Edge Detection Parameters ---
        self.canny_low_threshold = 50
        self.canny_high_threshold = 150
        self.canny_aperture_size = 3 # Must be 3, 5, or 7

        # --- Hough Line Transform Parameters ---
        self.hough_rho = 1.0          # Distance resolution in pixels
        self.hough_theta = np.pi / 180 # Angle resolution in radians
        self.hough_threshold = 50      # Minimum votes for a line
        self.hough_min_line_length = 30 # Minimum line length
        self.hough_max_line_gap = 10   # Maximum gap between line segments

        # --- ROI and Filtering ---
        self.vertical_scanlines = 20 # Number of vertical slices to analyze for average edge
        self.min_line_strength = 10  # Minimum accumulator value for line strength

        logger.debug("EdgeDetector initialized with default parameters.")

    def set_canny_parameters(self, low, high, aperture_size=3):
        """Set Canny edge detection parameters."""
        self.canny_low_threshold = low
        self.canny_high_threshold = high
        self.canny_aperture_size = aperture_size
        logger.debug("EdgeDetector Canny parameters updated.")

    def set_hough_parameters(self, rho, theta, threshold, min_len, max_gap):
        """Set HoughLinesP parameters."""
        self.hough_rho = rho
        self.hough_theta = theta
        self.hough_threshold = threshold
        self.hough_min_line_length = min_len
        self.hough_max_line_gap = max_gap
        logger.debug("EdgeDetector Hough parameters updated.")

    def detect_edges_and_lines(self, gray_frame: np.ndarray) -> List[Tuple[int, int, int, int, float, float, float]]:
        """
        Detect edges and lines in a grayscale frame.
        Args:
            gray_frame: Input grayscale image (ROI).
        Returns:
            List of tuples: (x1, y1, x2, y2, length, angle_rad, strength)
        """
        try:
            # 1. Apply Gaussian Blur to reduce noise (optional but often helpful)
            # blurred = cv2.GaussianBlur(gray_frame, (5, 5), 0)

            # 2. Canny Edge Detection
            edges = cv2.Canny(gray_frame, self.canny_low_threshold, self.canny_high_threshold, apertureSize=self.canny_aperture_size)

            # 3. Hough Line Transform (Probabilistic)
            lines = cv2.HoughLinesP(
                edges,
                rho=self.hough_rho,
                theta=self.hough_theta,
                threshold=self.hough_threshold,
                minLineLength=self.hough_min_line_length,
                maxLineGap=self.hough_max_line_gap
            )

            processed_lines = []
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    # Calculate line properties
                    length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                    angle = np.arctan2(y2 - y1, x2 - x1) # Angle in radians
                    # Strength could be the accumulator value (threshold), or length-based
                    strength = length # Simple strength measure

                    if strength >= self.min_line_strength:
                        processed_lines.append((x1, y1, x2, y2, length, angle, strength))
            
            return processed_lines

        except Exception as e:
            logger.error(f"EdgeDetector: Error detecting edges/lines: {e}")
            return []

    def calculate_average_edge_x(self, gray_frame: np.ndarray, side: str = 'both') -> Tuple[float, float, List[Tuple[int, int, int, int, float, float, float]]]:
        """
        Calculates the average X position of vertical edges (track boundaries).
        Args:
            gray_frame: Input grayscale image (ROI).
            side: 'left', 'right', or 'both'.
        Returns:
            Tuple: (avg_x: float, std_x: float, detected_lines: List[...])
                   avg_x is -1.0 if not found reliably.
        """
        detected_lines = self.detect_edges_and_lines(gray_frame)
        
        if not detected_lines:
            return -1.0, 0.0, detected_lines

        height, width = gray_frame.shape
        # Define vertical scanlines
        scanline_positions = np.linspace(0, width - 1, self.vertical_scanlines, dtype=int)
        
        edge_points_x = [] # Store X coordinates of detected edges

        for x_pos in scanline_positions:
            # For each vertical scanline, find intersections with detected lines
            for x1, y1, x2, y2, length, angle, strength in detected_lines:
                # Check if line segment crosses the vertical scanline
                if min(x1, x2) <= x_pos <= max(x1, x2) and abs(x2 - x1) > 1e-5: # Avoid division by zero
                    # Calculate Y intersection point
                    y_intersect = y1 + (x_pos - x1) * (y2 - y1) / (x2 - x1)
                    y_int = int(y_intersect)
                    # Check if intersection is within image bounds
                    if 0 <= y_int < height:
                        edge_points_x.append(x_pos) # We know the X, this is just confirming it's an edge
                        # A more precise method would interpolate the exact X on the line
                        # at the point where it crosses the scanline perpendicularly.
                        # For simplicity, we use the scanline X.

        if not edge_points_x:
            return -1.0, 0.0, detected_lines

        # Filter points based on side
        filtered_x = []
        if side == 'left':
            filtered_x = [x for x in edge_points_x if x < width * 0.4] # Arbitrary left 40%
        elif side == 'right':
            filtered_x = [x for x in edge_points_x if x > width * 0.6] # Arbitrary right 40%
        else: # both or any
            filtered_x = edge_points_x

        if not filtered_x:
             return -1.0, 0.0, detected_lines

        avg_x = float(np.mean(filtered_x))
        std_x = float(np.std(filtered_x))
        
        return avg_x, std_x, detected_lines


# Example usage (if run as script)
if __name__ == "__main__":
    # This would typically be tested with actual images
    print("EdgeDetector class defined.")

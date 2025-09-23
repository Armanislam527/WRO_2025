# vision/color_detector.py
"""Detects red and green traffic signs in an HSV image."""

import cv2
import numpy as np
import logging
from typing import Tuple, Optional

logger = logging.getLogger(__name__)

class ColorDetector:
    """
    Detects red and green traffic signs using HSV color thresholding and contour analysis.
    """

    def __init__(self):
        # --- Default HSV Ranges (NEEDS CALIBRATION) ---
        # These are starting points, should be calibrated per environment.
        # WRO Rules specify RGB(238, 39, 55) for Red and RGB(68, 214, 44) for Green.
        # Converting to approximate HSV ranges (using online converters or experimentation):
        
        # Red can be tricky due to HSV wrap-around (H near 0 and 180)
        self.lower_red1 = np.array([0, 100, 50])   # Lower range for red (H 0-10)
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 100, 50]) # Upper range for red (H 170-180)
        self.upper_red2 = np.array([180, 255, 255])

        self.lower_green = np.array([40, 100, 50])  # Lower range for green (H 40-80)
        self.upper_green = np.array([80, 255, 255]) # Upper range for green

        # --- Advanced Filtering Parameters ---
        self.min_area_threshold = 500.0  # Minimum contour area (pixels)
        self.min_aspect_ratio = 0.5      # Min width/height
        self.max_aspect_ratio = 2.0      # Max width/height
        self.min_solidity = 0.6          # Min ratio of contour area to its convex hull area
        self.min_extent = 0.4            # Min ratio of contour area to bounding rectangle area

        logger.debug("ColorDetector initialized with default parameters.")

    def set_hsv_ranges(self, lower_red1, upper_red1, lower_red2, upper_red2, lower_green, upper_green):
        """Set custom HSV ranges for detection."""
        self.lower_red1, self.upper_red1 = np.array(lower_red1), np.array(upper_red1)
        self.lower_red2, self.upper_red2 = np.array(lower_red2), np.array(upper_red2)
        self.lower_green, self.upper_green = np.array(lower_green), np.array(upper_green)
        logger.debug("ColorDetector HSV ranges updated.")

    def set_filtering_parameters(self, min_area=None, min_ar=None, max_ar=None, min_solidity=None, min_extent=None):
        """Set advanced filtering parameters."""
        if min_area is not None: self.min_area_threshold = min_area
        if min_ar is not None: self.min_aspect_ratio = min_ar
        if max_ar is not None: self.max_aspect_ratio = max_ar
        if min_solidity is not None: self.min_solidity = min_solidity
        if min_extent is not None: self.min_extent = min_extent
        logger.debug("ColorDetector filtering parameters updated.")

    def _filter_contour(self, contour) -> bool:
        """
        Apply advanced geometric filters to a contour.
        Args:
            contour: The contour to filter.
        Returns:
            bool: True if the contour passes the filters, False otherwise.
        """
        area = cv2.contourArea(contour)
        if area < self.min_area_threshold:
            return False

        x, y, w, h = cv2.boundingRect(contour)
        aspect_ratio = float(w) / h if h > 0 else 0
        if not (self.min_aspect_ratio <= aspect_ratio <= self.max_aspect_ratio):
            return False

        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        if hull_area > 0:
            solidity = float(area) / hull_area
            if solidity < self.min_solidity:
                return False

        rect_area = w * h
        if rect_area > 0:
            extent = float(area) / rect_area
            if extent < self.min_extent:
                return False

        # Add more filters if needed (e.g., circularity for circles)
        return True

    def detect_red_sign(self, hsv_frame: np.ndarray) -> Tuple[bool, Optional[Tuple[int, int]], float, Tuple[int, int, int, int], int]:
        """
        Detect a red traffic sign in the HSV frame.
        Args:
            hsv_frame: The input image in HSV color space.
        Returns:
            Tuple: (found: bool, centroid: (x,y) or None, confidence: float, bbox: (x,y,w,h), area: int)
        """
        try:
            # Threshold the HSV image to get only red colors
            mask1 = cv2.inRange(hsv_frame, self.lower_red1, self.upper_red1)
            mask2 = cv2.inRange(hsv_frame, self.lower_red2, self.upper_red2)
            mask_red = cv2.bitwise_or(mask1, mask2)

            # Find contours
            contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            best_area = 0
            best_contour = None
            for cnt in contours:
                if self._filter_contour(cnt):
                    area = cv2.contourArea(cnt)
                    if area > best_area:
                        best_area = area
                        best_contour = cnt

            if best_contour is not None:
                # Calculate centroid using moments
                m = cv2.moments(best_contour)
                if m["m00"] != 0:
                    cx = int(m["m10"] / m["m00"])
                    cy = int(m["m01"] / m["m00"])
                    centroid = (cx, cy)
                else:
                    # Fallback centroid
                    x, y, w, h = cv2.boundingRect(best_contour)
                    centroid = (x + w // 2, y + h // 2)

                x, y, w, h = cv2.boundingRect(best_contour)
                bbox = (x, y, w, h)
                
                # Simple confidence based on area (can be refined)
                # Normalize by image area or use a max expected area
                confidence = min(1.0, best_area / 5000.0) # Example normalization

                return True, centroid, confidence, bbox, int(best_area)

            return False, None, 0.0, (0, 0, 0, 0), 0

        except Exception as e:
            logger.error(f"ColorDetector: Error detecting red sign: {e}")
            return False, None, 0.0, (0, 0, 0, 0), 0

    def detect_green_sign(self, hsv_frame: np.ndarray) -> Tuple[bool, Optional[Tuple[int, int]], float, Tuple[int, int, int, int], int]:
        """
        Detect a green traffic sign in the HSV frame.
        Args:
            hsv_frame: The input image in HSV color space.
        Returns:
            Tuple: (found: bool, centroid: (x,y) or None, confidence: float, bbox: (x,y,w,h), area: int)
        """
        try:
            # Threshold the HSV image to get only green colors
            mask_green = cv2.inRange(hsv_frame, self.lower_green, self.upper_green)

            # Find contours
            contours, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            best_area = 0
            best_contour = None
            for cnt in contours:
                if self._filter_contour(cnt):
                    area = cv2.contourArea(cnt)
                    if area > best_area:
                        best_area = area
                        best_contour = cnt

            if best_contour is not None:
                m = cv2.moments(best_contour)
                if m["m00"] != 0:
                    cx = int(m["m10"] / m["m00"])
                    cy = int(m["m01"] / m["m00"])
                    centroid = (cx, cy)
                else:
                    x, y, w, h = cv2.boundingRect(best_contour)
                    centroid = (x + w // 2, y + h // 2)

                x, y, w, h = cv2.boundingRect(best_contour)
                bbox = (x, y, w, h)
                
                confidence = min(1.0, best_area / 5000.0) # Example normalization

                return True, centroid, confidence, bbox, int(best_area)

            return False, None, 0.0, (0, 0, 0, 0), 0

        except Exception as e:
            logger.error(f"ColorDetector: Error detecting green sign: {e}")
            return False, None, 0.0, (0, 0, 0, 0), 0

# Example usage (if run as script)
if __name__ == "__main__":
    # This would typically be tested with actual images
    print("ColorDetector class defined.")

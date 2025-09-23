# navigation/positional_memory.py
"""Stores and compares positional data for start/stop and parking logic."""

import logging
from typing import TYPE_CHECKING, Optional
import numpy as np

if TYPE_CHECKING:
    from sensors.sensor_data import SensorData
    from vision.vision_snapshot import VisionSnapshot

logger = logging.getLogger(__name__)

class PositionalMemory:
    """
    Stores sensor and vision snapshots at key locations (start, pre-stop, parking entrance).
    Compares current state to these snapshots to determine location.
    Mirrors the C++ PositionalMemory class.
    """

    def __init__(self):
        self.initial_data_stored = False
        self.pre_stop_data_stored = False
        self.parking_entrance_data_stored = False

        self.initial_sensor_data: Optional['SensorData'] = None
        self.initial_vision_snapshot: Optional['VisionSnapshot'] = None

        self.pre_stop_sensor_data: Optional['SensorData'] = None
        self.pre_stop_vision_snapshot: Optional['VisionSnapshot'] = None

        self.parking_entrance_sensor_data: Optional['SensorData'] = None
        self.parking_entrance_vision_snapshot: Optional['VisionSnapshot'] = None

        logger.debug("PositionalMemory initialized.")

    def store_initial_position(self, sensor_snapshot: 'SensorData', vision_snapshot: 'VisionSnapshot'):
        """
        Store the initial state when the mission officially starts (after GO signal).
        This is the reference point for returning after 3 laps.
        Args:
            sensor_snapshot: Sensor data at the start.
            vision_snapshot: Vision data at the start.
        """
        self.initial_sensor_data = sensor_snapshot
        self.initial_vision_snapshot = vision_snapshot
        self.initial_data_stored = True
        logger.info("PositionalMemory: Initial position stored.")

    def store_pre_stop_position(self, sensor_snapshot: 'SensorData', vision_snapshot: 'VisionSnapshot'):
        """
        Store the position just before attempting to stop in the start section.
        This might be slightly different from initial due to sensor drift or initial movement.
        Args:
            sensor_snapshot: Sensor data at pre-stop.
            vision_snapshot: Vision data at pre-stop.
        """
        self.pre_stop_sensor_data = sensor_snapshot
        self.pre_stop_vision_snapshot = vision_snapshot
        self.pre_stop_data_stored = True
        logger.info("PositionalMemory: Pre-stop position stored.")

    def store_parking_entrance_position(self, sensor_snapshot: 'SensorData', vision_snapshot: 'VisionSnapshot'):
        """
        Store the position when entering the parking lot area (Obstacle challenge).
        Args:
            sensor_snapshot: Sensor data at parking entrance.
            vision_snapshot: Vision data at parking entrance.
        """
        self.parking_entrance_sensor_data = sensor_snapshot
        self.parking_entrance_vision_snapshot = vision_snapshot
        self.parking_entrance_data_stored = True
        logger.info("PositionalMemory: Parking entrance position stored.")

    def compare_with_initial(self, current_sensor_data: 'SensorData', current_vision_data: 'VisionSnapshot') -> float:
        """
        Compare current state with stored initial state.
        Args:
            current_sensor_data: Current sensor data.
            current_vision_data: Current vision data.
        Returns:
            float: Similarity score (0.0 to 1.0, 1.0 = identical).
        """
        if not self.initial_data_stored:
            return 0.0
        sensor_sim = self._calculate_sensor_similarity(self.initial_sensor_data, current_sensor_data)
        vision_sim = self._calculate_vision_similarity(self.initial_vision_snapshot, current_vision_data)
        # Simple average, could be weighted
        return (sensor_sim + vision_sim) / 2.0

    def compare_with_pre_stop(self, current_sensor_data: 'SensorData', current_vision_data: 'VisionSnapshot') -> float:
        """
        Compare current state with stored pre-stop state.
        Args:
            current_sensor_data: Current sensor data.
            current_vision_data: Current vision data.
        Returns:
            float: Similarity score (0.0 to 1.0, 1.0 = identical).
        """
        if not self.pre_stop_data_stored:
            return 0.0
        sensor_sim = self._calculate_sensor_similarity(self.pre_stop_sensor_data, current_sensor_data)
        vision_sim = self._calculate_vision_similarity(self.pre_stop_vision_snapshot, current_vision_data)
        return (sensor_sim + vision_sim) / 2.0

    def compare_with_parking_entrance(self, current_sensor_data: 'SensorData', current_vision_data: 'VisionSnapshot') -> float:
        """
        Compare current state with stored parking entrance state.
        Args:
            current_sensor_data: Current sensor data.
            current_vision_data: Current vision data.
        Returns:
            float: Similarity score (0.0 to 1.0, 1.0 = identical).
        """
        if not self.parking_entrance_data_stored:
            return 0.0
        sensor_sim = self._calculate_sensor_similarity(self.parking_entrance_sensor_data, current_sensor_data)
        vision_sim = self._calculate_vision_similarity(self.parking_entrance_vision_snapshot, current_vision_data)
        return (sensor_sim + vision_sim) / 2.0

    def is_likely_at_initial_position(self, current_sensor_data: 'SensorData', current_vision_data: 'VisionSnapshot', threshold: float = 0.8) -> bool:
        """
        Check if we are likely back at the initial position (for stopping/parking).
        Args:
            current_sensor_data: Current sensor data.
            current_vision_data: Current vision data.
            threshold: Similarity threshold (0.0 to 1.0).
        Returns:
            bool: True if similarity is above threshold.
        """
        similarity = self.compare_with_initial(current_sensor_data, current_vision_data)
        is_likely = similarity >= threshold
        if is_likely:
            logger.debug(f"PositionalMemory: Likely at INITIAL position (Sim: {similarity:.2f} >= {threshold}).")
        return is_likely

    def is_likely_at_pre_stop_position(self, current_sensor_data: 'SensorData', current_vision_data: 'VisionSnapshot', threshold: float = 0.9) -> bool:
        """
        Check if we are likely at the pre-stop position (for fine-tuning stop).
        Args:
            current_sensor_data: Current sensor data.
            current_vision_data: Current vision data.
            threshold: Similarity threshold (0.0 to 1.0).
        Returns:
            bool: True if similarity is above threshold.
        """
        similarity = self.compare_with_pre_stop(current_sensor_data, current_vision_data)
        is_likely = similarity >= threshold
        if is_likely:
            logger.debug(f"PositionalMemory: Likely at PRE-STOP position (Sim: {similarity:.2f} >= {threshold}).")
        return is_likely

    def is_likely_at_parking_entrance(self, current_sensor_data: 'SensorData', current_vision_data: 'VisionSnapshot', threshold: float = 0.8) -> bool:
        """
        Check if we are likely at the parking entrance (for initiating parking).
        Args:
            current_sensor_data: Current sensor data.
            current_vision_data: Current vision data.
            threshold: Similarity threshold (0.0 to 1.0).
        Returns:
            bool: True if similarity is above threshold.
        """
        similarity = self.compare_with_parking_entrance(current_sensor_data, current_vision_data)
        is_likely = similarity >= threshold
        if is_likely:
            logger.debug(f"PositionalMemory: Likely at PARKING ENTRANCE (Sim: {similarity:.2f} >= {threshold}).")
        return is_likely

    # --- Helper Functions for Similarity Calculation ---
    # These are simplified examples. Real implementations would be more sophisticated.

    def _calculate_sensor_similarity(self, data1: 'SensorData', data2: 'SensorData') -> float:
        """Calculate similarity between two SensorData objects."""
        if not data1 or not data2:
            return 0.0
        try:
            # Example: Compare US sensor distances
            us1 = data1.us_data
            us2 = data2.us_data
            if not us1 or not us2:
                return 0.0

            # Normalize differences (e.g., by max expected distance 200cm)
            max_dist = 200.0
            diff_front = abs(us1.front_distance - us2.front_distance) / max_dist
            diff_right = abs(us1.right_distance - us2.right_distance) / max_dist
            diff_back = abs(us1.back_distance - us2.back_distance) / max_dist
            diff_left = abs(us1.left_distance - us2.left_distance) / max_dist

            # Similarity is 1 - normalized difference (clamped)
            sim_front = max(0.0, 1.0 - diff_front)
            sim_right = max(0.0, 1.0 - diff_right)
            sim_back = max(0.0, 1.0 - diff_back)
            sim_left = max(0.0, 1.0 - diff_left)

            # Average similarity
            return (sim_front + sim_right + sim_back + sim_left) / 4.0
        except Exception as e:
            logger.error(f"PositionalMemory: Error calculating sensor similarity: {e}")
            return 0.0

    def _calculate_vision_similarity(self, data1: 'VisionSnapshot', data2: 'VisionSnapshot') -> float:
        """Calculate similarity between two VisionSnapshot objects."""
        if not data1 or not data2:
            return 0.0
        try:
            # Example: Compare average edge positions
            # Normalize by frame width
            import config.vehicle_config as cfg
            frame_width = cfg.CAMERA_WIDTH
            if frame_width <= 0:
                return 0.0

            # Handle cases where edge is not detected (-1.0)
            # If both are -1, consider them similar for that edge.
            # If one is -1 and the other isn't, not similar.
            max_edge_diff = frame_width

            def edge_sim(e1, e2):
                if e1 < 0 and e2 < 0:
                    return 1.0 # Both not detected
                elif e1 < 0 or e2 < 0:
                    return 0.0 # One detected, one not
                else:
                    return max(0.0, 1.0 - (abs(e1 - e2) / max_edge_diff))

            edge_left_sim = edge_sim(data1.avg_left_edge_x, data2.avg_left_edge_x)
            edge_right_sim = edge_sim(data1.avg_right_edge_x, data2.avg_right_edge_x)

            # Example: Compare presence of primary signs
            sign_sim = 0.0
            data1_has_left = data1.primary_left_sign is not None
            data1_has_right = data1.primary_right_sign is not None
            data2_has_left = data2.primary_left_sign is not None
            data2_has_right = data2.primary_right_sign is not None

            # Simple check: if both have same sign presence, 1.0, if different, 0.0
            # More nuanced logic could compare sign types/positions
            if data1_has_left == data2_has_left and data1_has_right == data2_has_right:
                sign_sim = 1.0
            # Could add partial credit for one matching sign

            # Combine edge and sign similarity (weighted average)
            total_sim = (edge_left_sim + edge_right_sim + sign_sim) / 3.0 # 3 components
            return total_sim

        except Exception as e:
            logger.error(f"PositionalMemory: Error calculating vision similarity: {e}")
            return 0.0

    # Getters for stored data (useful for debugging or navigation)
    def has_initial_data(self) -> bool:
        return self.initial_data_stored

    def has_pre_stop_data(self) -> bool:
        return self.pre_stop_data_stored

    def has_parking_entrance_data(self) -> bool:
        return self.parking_entrance_data_stored

    def get_initial_sensor_data(self) -> Optional['SensorData']:
        return self.initial_sensor_data

    def get_initial_vision_data(self) -> Optional['VisionSnapshot']:
        return self.initial_vision_snapshot

# Example usage (if run as script)
if __name__ == "__main__":
    import utils.logger
    utils.logger.setup_logging()
    pm = PositionalMemory()
    print("PositionalMemory class defined.")

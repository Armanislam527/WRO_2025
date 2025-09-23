# navigation/lap_counter.py
"""Counts laps and determines if the vehicle is in the start section."""

import logging
import time
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from sensors.sensor_data import SensorData
    from vision.vision_snapshot import VisionSnapshot
    from navigation.positional_memory import PositionalMemory # Add this import

import config.vehicle_config as cfg

logger = logging.getLogger(__name__)

class LapCounter:
    """
    Counts laps and determines if the vehicle is in the start section.
    Uses IMU odometry (distance/angle) and PositionalMemory for robustness.
    """

    def __init__(self):
        self.current_lap = 0
        self.lap_completed_flag = False
        
        # --- Odometry State ---
        self._total_distance_traveled_cm = 0.0 # In cm
        self._total_angle_turned_deg = 0.0 # In degrees (cumulative, can be >360)
        self._last_sensor_data: Optional['SensorData'] = None
        self._last_update_time: Optional[float] = None

        # --- Thresholds (Tunable) ---
        # These represent a full lap. They are fallbacks or sanity checks.
        # Primary lap detection uses PositionalMemory.
        self._lap_distance_threshold_cm = 3000.0 # Example: 30 meters track perimeter estimate
        self._lap_angle_threshold_deg = 400.0   # Slightly > 360 to account for overshoot/oscillation

        # --- Positional Memory Reference ---
        self._positional_memory: Optional['PositionalMemory'] = None

        logger.debug("LapCounter initialized.")

    def initialize(self, positional_memory: Optional['PositionalMemory'] = None) -> bool:
        """
        Initialize the LapCounter with dependencies.
        Args:
            positional_memory: Reference to PositionalMemory for comparison.
        """
        self._positional_memory = positional_memory
        self.reset()
        logger.info("LapCounter initialized with dependencies.")
        return True

    def reset(self):
        """Reset the counter to zero laps and clear odometry state."""
        self.current_lap = 0
        self.lap_completed_flag = False
        self._total_distance_traveled_cm = 0.0
        self._total_angle_turned_deg = 0.0
        self._last_sensor_data = None
        self._last_update_time = None
        logger.debug("LapCounter reset.")

    def update(self, sensor_data: 'SensorData', vision_snapshot: Optional['VisionSnapshot'] = None):
        """
        Update the lap counter based on new sensor data (IMU odometry).
        Checks for lap completion using PositionalMemory if available.
        Args:
            sensor_data: Latest sensor data.
            vision_snapshot: Latest vision data (optional, for future enhancements).
        """
        if not sensor_data or not sensor_data.imu_data:
            logger.warning("LapCounter: update() called with invalid sensor_data.")
            return

        current_time = time.time()

        # --- 1. Update Odometry (Distance and Angle) ---
        if self._last_sensor_data is not None and self._last_sensor_data.imu_data:
            delta_t = current_time - (self._last_update_time if self._last_update_time else current_time)
            if delta_t > 0: # Prevent division by zero or negative time
                # --- Distance Estimation ---
                # Use accelerometer data for more accurate odometry than US sensor changes.
                # a_x, a_y are in g. Convert to cm/s^2: 1g = 981 cm/s^2
                # Integrate acceleration twice to get distance: s = 0.5 * a * t^2 (assuming initial vel is small)
                # This is a simplified model. Real implementation needs velocity estimation and filtering.
                # For now, a basic approximation using average acceleration:
                avg_accel_x_g = (self._last_sensor_data.imu_data.accel_x + sensor_data.imu_data.accel_x) / 2.0
                avg_accel_y_g = (self._last_sensor_data.imu_data.accel_y + sensor_data.imu_data.accel_y) / 2.0
                avg_accel_x_cm_s2 = avg_accel_x_g * 981.0
                avg_accel_y_cm_s2 = avg_accel_y_g * 981.0
                
                # Estimate distance change (assuming motion is primarily along vehicle's X or Y axis)
                # This is still quite approximate. A better model would transform to world frame or use gyro.
                delta_dist_x_cm = 0.5 * avg_accel_x_cm_s2 * (delta_t ** 2)
                delta_dist_y_cm = 0.5 * avg_accel_y_cm_s2 * (delta_t ** 2)
                # Use the component that likely represents forward motion (depends on vehicle orientation)
                # Let's assume X is forward/backward for simplicity here.
                distance_increment_cm = abs(delta_dist_x_cm) # Use absolute to accumulate total distance
                
                # --- Angle Estimation ---
                # Integrate gyro Z (yaw rate) over time to get change in heading.
                # gyro_z is in deg/s.
                avg_gyro_z_deg_s = (self._last_sensor_data.imu_data.gyro_z + sensor_data.imu_data.gyro_z) / 2.0
                angle_increment_deg = avg_gyro_z_deg_s * delta_t

                # Update cumulative odometry
                self._total_distance_traveled_cm += distance_increment_cm
                self._total_angle_turned_deg += angle_increment_deg
                # logger.debug(f"LapCounter: Dist Inc: {distance_increment_cm:.2f}cm, Angle Inc: {angle_increment_deg:.2f}deg")

        # Update last state
        self._last_sensor_data = sensor_data
        self._last_update_time = current_time

        # --- 2. Check for Lap Completion (Primary Method: PositionalMemory) ---
        # Only check if we have stored initial data and are still counting laps (<3)
        if (self._positional_memory and self._positional_memory.has_initial_data() and
            not self.lap_completed_flag and self.current_lap < 3):

            # Compare current state with the stored initial state
            similarity = self._positional_memory.compare_with_initial(sensor_data, vision_snapshot)
            
            # Define a threshold for being "back at start"
            # This needs tuning. High threshold (e.g., 0.95) means very confident.
            lap_completion_similarity_threshold = 0.92 # Tunable parameter

            if similarity > lap_completion_similarity_threshold:
                # We are likely back at the start. Check odometry as a sanity check.
                odometry_suggests_lap = (
                    self._total_distance_traveled_cm > (self._lap_distance_threshold_cm * 0.7) or
                    abs(self._total_angle_turned_deg) > (self._lap_angle_threshold_deg * 0.7)
                )
                
                if odometry_suggests_lap:
                    logger.info(f"LapCounter: Lap {self.current_lap + 1} completion detected! "
                                f"(Sim: {similarity:.2f}, Dist: {self._total_distance_traveled_cm:.1f}cm, "
                                f"Angle: {self._total_angle_turned_deg:.1f}deg)")
                    self.lap_completed_flag = True # Set flag for main loop
                else:
                    logger.debug(f"LapCounter: High similarity ({similarity:.2f}) but odometry low. "
                                 f"Waiting for more movement evidence.")

        # --- 3. Fallback Check (if PositionalMemory fails or is not ready) ---
        # This is less robust but provides a backup.
        # elif (not self.lap_completed_flag and self.current_lap < 3 and
        #       (self._total_distance_traveled_cm > self._lap_distance_threshold_cm or
        #        abs(self._total_angle_turned_deg) > self._lap_angle_threshold_deg)):
        #     logger.info(f"LapCounter: Fallback Lap {self.current_lap + 1} completion detected! "
        #                 f"(Dist: {self._total_distance_traveled_cm:.1f}cm, "
        #                 f"Angle: {self._total_angle_turned_deg:.1f}deg)")
        #     self.lap_completed_flag = True

    def is_lap_completed(self) -> bool:
        """Check if a lap completion has been detected."""
        return self.lap_completed_flag

    def acknowledge_lap_completion(self):
        """Acknowledge that a lap completion has been processed."""
        if self.lap_completed_flag:
            self.lap_completed_flag = False
            self.current_lap += 1
            # Reset odometry counters for the next lap
            self._total_distance_traveled_cm = 0.0
            self._total_angle_turned_deg = 0.0
            logger.info(f"LapCounter: Lap completion for lap {self.current_lap} acknowledged.")

    def get_current_lap(self) -> int:
        """Get the current lap number (0-based, 0, 1, 2)."""
        return self.current_lap

    # Note: is_in_start_section logic is primarily handled by PositionalMemory in main.py
    # based on the stored initial/pre-stop data. This class focuses on lap counting.

# Example usage (if run as script)
if __name__ == "__main__":
    import utils.logger
    utils.logger.setup_logging()
    lc = LapCounter()
    print("LapCounter class defined.")

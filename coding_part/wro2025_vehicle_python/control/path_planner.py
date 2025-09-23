# control/path_planner.py
"""Advanced Path Planner, mirroring the C++ structure and logic."""

import logging
from typing import TYPE_CHECKING
from navigation.mission_state import MissionState, MissionPhase
# Import our data structures
from control.navigation_command import NavigationCommand, NavigationAction
# Use TYPE_CHECKING to avoid circular imports during type checking
if TYPE_CHECKING:
    from sensors.sensor_data import SensorData
    from vision.vision_snapshot import VisionSnapshot
    from navigation.mission_state import MissionState
    from navigation.lap_counter import LapCounter
    # from vision.sign_detector import SignDetector # If we had a separate sign detector

logger = logging.getLogger(__name__)

class PathPlanner:
    """
    Plans high-level navigation commands based on sensor data, vision data, and mission state.
    Mirrors the C++ PathPlanner class structure.
    """

    def __init__(self):
        """Initialize the PathPlanner."""
        self.last_known_challenge = None # To track changes
        # Dependencies (to be injected or initialized later)
        self.lap_counter: 'LapCounter' = None # Placeholder
        # self.sign_detector = None # If separate

        logger.info("PathPlanner initialized.")

    def initialize(self, lap_counter: 'LapCounter'): # , sign_detector=None): # Add sign_detector if needed
        """
        Initialize the PathPlanner with its dependencies.
        Args:
            lap_counter: An instance of LapCounter.
            # sign_detector: An instance of SignDetector (optional).
        Returns:
            bool: True if initialization is successful.
        """
        if not lap_counter: # or not sign_detector):
            logger.error("PathPlanner: Invalid dependencies provided.")
            return False

        self.lap_counter = lap_counter
        # self.sign_detector = sign_detector
        logger.info("PathPlanner: Initialized with dependencies.")
        return True

    def plan(self, sensor_data: 'SensorData', vision_snapshot: 'VisionSnapshot', mission_state: 'MissionState') -> NavigationCommand:
        """
        Main planning function. Dispatches to specific planning methods based on mission state.
        Args:
            sensor_data: Latest sensor data.
            vision_snapshot: Latest vision processing results.
            mission_state: Current mission state.
        Returns:
            NavigationCommand: The planned command.
        """
        # Update internal state if challenge type changed
        current_challenge = mission_state.get_challenge_type()
        logger.debug(f"PathPlanner.plan() called with challenge type: {current_challenge}") # Add this line for debugging
        if current_challenge != self.last_known_challenge:
            self.last_known_challenge = current_challenge
            challenge_name = "Open" if mission_state.is_open_challenge() else "Obstacle"
            logger.info(f"PathPlanner: Challenge type changed to {challenge_name}")

        # Dispatch to specific planning logic based on mission phase and challenge type
        phase = mission_state.get_phase()
        try:
            if phase == MissionPhase.DRIVING_LAPS:
                if mission_state.is_open_challenge():
                    return self._plan_for_open_challenge(sensor_data, vision_snapshot, mission_state)
                elif mission_state.is_obstacle_challenge():
                    return self._plan_for_obstacle_challenge(sensor_data, vision_snapshot, mission_state)

            elif phase in (MissionPhase.STOPPING_AT_START,):
                # This phase is for stopping in the start section after 3 laps.
                # The main logic for *when* to stop should be in main loop using PositionalMemory.
                # PathPlanner provides the command to execute the stop.
                return self._plan_for_stopping(sensor_data, vision_snapshot, mission_state)

            elif phase in (MissionPhase.FINDING_PARKING, MissionPhase.PARKING):
                if mission_state.is_obstacle_challenge():
                    return self._plan_for_parking(sensor_data, vision_snapshot, mission_state)

            elif phase == MissionPhase.EMERGENCY_STOPPED:
                return NavigationCommand(NavigationAction.EMERGENCY_STOP, 0.0)

            else:
                # Phases like PRE_START, WAITING_FOR_GO, COMPLETED don't require path planning here
                # Default action if no specific logic applies
                logger.debug(f"PathPlanner: No specific plan for phase {phase}. Defaulting to GO_STRAIGHT.")
                return NavigationCommand(NavigationAction.GO_STRAIGHT, 0.0)

        except Exception as e:
            logger.error(f"PathPlanner: Error during planning: {e}", exc_info=True)
            # Fail-safe: Emergency stop on planning error
            return NavigationCommand(NavigationAction.EMERGENCY_STOP, 0.0)

    def _plan_for_open_challenge(self, sensor_data: 'SensorData', vision_snapshot: 'VisionSnapshot', mission_state: 'MissionState') -> NavigationCommand:
        """
        Plan path for the Open Challenge (no traffic signs, follow walls/center line).
        Args:
            sensor_data: Latest sensor data.
            vision_snapshot: Latest vision processing results.
            mission_state: Current mission state.
        Returns:
            NavigationCommand: The planned command.
        """
        logger.debug("PathPlanner: Planning for Open Challenge.")
        desired_speed = 0.6 # Base speed

        # --- Basic Obstacle Avoidance (Fallback/Supplement) ---
        # This takes precedence over normal navigation
        avoidance_needed, turn_direction = self._needs_obstacle_avoidance(sensor_data)
        if avoidance_needed:
            avoidance_speed = 0.4 # Slow down
            if turn_direction < 0: # Turn left
                logger.info("PathPlanner (Open): Obstacle avoidance - turning left.")
                # For simplicity, assume steering is handled by lower level based on US
                return NavigationCommand(NavigationAction.GO_STRAIGHT, avoidance_speed)
            elif turn_direction > 0: # Turn right
                logger.info("PathPlanner (Open): Obstacle avoidance - turning right.")
                return NavigationCommand(NavigationAction.GO_STRAIGHT, avoidance_speed)
            else: # Go straight (e.g., center obstacle, back up?)
                 logger.info("PathPlanner (Open): Obstacle avoidance - going straight/backing up?.")
                 return NavigationCommand(NavigationAction.GO_STRAIGHT, -avoidance_speed) # Negative for reverse?

        # --- Wall Following / Center Line Following Logic ---
        # Use vision data (edge detection) or sensor data (US distances) for wall following.
        # Simple Proportional control based on vision edge data
        left_edge_x = vision_snapshot.avg_left_edge_x
        right_edge_x = vision_snapshot.avg_right_edge_x
        track_width_estimate = vision_snapshot.get_track_width_estimate()

        # Assume camera resolution width is known (from config)
        import config.vehicle_config as cfg
        frame_center_x = cfg.CAMERA_WIDTH / 2.0

        # Strategy: Follow center of track
        if left_edge_x >= 0 and right_edge_x >= 0 and track_width_estimate > 50: # Valid edges
            track_center_x = (left_edge_x + right_edge_x) / 2.0
            error = track_center_x - frame_center_x

            # Simple P controller (Kp needs tuning)
            Kp = 0.01 # Tuning parameter
            steering_adjustment = -Kp * error # Negative because positive error means turn right

            # Map steering_adjustment to servo angle or command
            # This is conceptual; actual mapping happens in VehicleController
            # For command, we might just indicate intent and speed
            # A more advanced system might calculate a specific steering angle/servo command here

            if abs(error) < 20: # Within tolerance
                logger.debug(f"PathPlanner (Open): Centered (Error: {error:.1f}). Going straight.")
                return NavigationCommand(NavigationAction.GO_STRAIGHT, desired_speed)
            elif error > 0: # Too far right, need to turn left
                logger.debug(f"PathPlanner (Open): Off-center (Error: {error:.1f}). Turning left.")
                return NavigationCommand(NavigationAction.TURN_LEFT, desired_speed)
            else: # error < 0, Too far left, need to turn right
                logger.debug(f"PathPlanner (Open): Off-center (Error: {error:.1f}). Turning right.")
                return NavigationCommand(NavigationAction.TURN_RIGHT, desired_speed)

        elif left_edge_x >= 0: # Only left edge visible, follow left wall
             logger.debug("PathPlanner (Open): Following left wall.")
             return NavigationCommand(NavigationAction.FOLLOW_LEFT_LANE, desired_speed)
        elif right_edge_x >= 0: # Only right edge visible, follow right wall
             logger.debug("PathPlanner (Open): Following right wall.")
             return NavigationCommand(NavigationAction.FOLLOW_RIGHT_LANE, desired_speed)
        else:
            # No clear edges, go straight cautiously
            logger.warning("PathPlanner (Open): No clear track edges detected. Proceeding straight.")
            return NavigationCommand(NavigationAction.GO_STRAIGHT, desired_speed * 0.8) # Slow down

    def _plan_for_obstacle_challenge(self, sensor_data: 'SensorData', vision_snapshot: 'VisionSnapshot', mission_state: 'MissionState') -> NavigationCommand:
        """
        Plan path for the Obstacle Challenge (follow traffic signs).
        Args:
            sensor_data: Latest sensor data.
            vision_snapshot: Latest vision processing results.
            mission_state: Current mission state.
        Returns:
            NavigationCommand: The planned command.
        """
        logger.debug("PathPlanner: Planning for Obstacle Challenge.")
        desired_speed = 0.6 # Base speed

        # --- Basic Obstacle Avoidance (Fallback/Supplement) ---
        # Takes precedence even in Obstacle Challenge
        avoidance_needed, turn_direction = self._needs_obstacle_avoidance(sensor_data)
        if avoidance_needed:
             avoidance_speed = 0.4
             if turn_direction < 0:
                 logger.info("PathPlanner (Obstacle): Obstacle avoidance - turning left.")
                 return NavigationCommand(NavigationAction.GO_STRAIGHT, avoidance_speed)
             elif turn_direction > 0:
                 logger.info("PathPlanner (Obstacle): Obstacle avoidance - turning right.")
                 return NavigationCommand(NavigationAction.GO_STRAIGHT, avoidance_speed)
             else:
                 logger.info("PathPlanner (Obstacle): Obstacle avoidance - going straight.")
                 return NavigationCommand(NavigationAction.GO_STRAIGHT, -avoidance_speed)

        # --- Sign Following Logic ---
        # Determine lane to follow based on sign detection
        primary_left_sign = vision_snapshot.primary_left_sign
        primary_right_sign = vision_snapshot.primary_right_sign

        follow_left = False
        follow_right = False

        # Logic: if both signs detected with high confidence, it's a conflict or close proximity.
        # Prioritize based on confidence or last known rule (e.g., last sign seen?).
        # For simplicity, let's prioritize the one with higher confidence, or left if equal/both low.
        # A more robust system would need better conflict resolution.

        if primary_left_sign and primary_left_sign.is_valid() and primary_left_sign.confidence > 0.5:
            follow_left = True
        if primary_right_sign and primary_right_sign.is_valid() and primary_right_sign.confidence > 0.5:
            follow_right = True

        if follow_left and follow_right:
            # Conflict: Both signs detected. Resolve.
            logger.warning("PathPlanner (Obstacle): Potential sign conflict detected.")
            if primary_left_sign.confidence > primary_right_sign.confidence:
                follow_right = False
                logger.info("PathPlanner (Obstacle): Resolved conflict, following LEFT sign (higher confidence).")
            elif primary_right_sign.confidence > primary_left_sign.confidence:
                follow_left = False
                logger.info("PathPlanner (Obstacle): Resolved conflict, following RIGHT sign (higher confidence).")
            else:
                # Equal confidence or both low, default to left or last known direction?
                # Let's default to left for now.
                follow_right = False
                logger.info("PathPlanner (Obstacle): Resolved conflict, defaulting to LEFT sign.")

        if follow_left:
            logger.info("PathPlanner (Obstacle): Following LEFT lane as instructed by sign.")
            return NavigationCommand(NavigationAction.FOLLOW_LEFT_LANE, desired_speed)
        elif follow_right:
            logger.info("PathPlanner (Obstacle): Following RIGHT lane as instructed by sign.")
            return NavigationCommand(NavigationAction.FOLLOW_RIGHT_LANE, desired_speed)
        else:
            # No clear sign instruction. Default behavior?
            # Could be: follow center, follow last known direction, or wait/slow down.
            # Let's assume follow center line based on edges if available, otherwise slow down.
            logger.info("PathPlanner (Obstacle): No clear sign instruction. Following center line or slowing down.")
            # Re-use open challenge centering logic as fallback
            return self._plan_for_open_challenge(sensor_data, vision_snapshot, mission_state) # Or a simplified version


    def _plan_for_stopping(self, sensor_data: 'SensorData', vision_snapshot: 'VisionSnapshot', mission_state: 'MissionState') -> NavigationCommand:
        """
        Plan for stopping in the start section (Open) or finishing (Obstacle).
        The decision to stop is made externally (e.g., by PositionalMemory in main loop).
        This just provides the command to execute the stop.
        Args:
            sensor_data: Latest sensor data.
            vision_snapshot: Latest vision processing results.
            mission_state: Current mission state.
        Returns:
            NavigationCommand: The STOP command.
        """
        logger.info("PathPlanner: Generating STOP command for stopping phase.")
        # The action (STOP_IN_START) implies stopping and potentially specific maneuver
        # The speed is 0.0
        return NavigationCommand(NavigationAction.STOP_IN_START, 0.0)

    def _plan_for_parking(self, sensor_data: 'SensorData', vision_snapshot: 'VisionSnapshot', mission_state: 'MissionState') -> NavigationCommand:
        """
        Plan for finding and executing parking (Obstacle Challenge).
        """
        # ... (previous code) ...

        # --- Refine Parking Logic ---
        if phase == mission_state.MissionPhase.FINDING_PARKING:
            logger.debug("PathPlanner: In FINDING_PARKING phase.")
            # Simplified check: Assume we are in the start section and looking for the spot.
            # A more robust check would use positional memory or vision to confirm proximity to the parking area.
            # The C++ code has a placeholder check for large right distance.
            # Let's use a similar simple check for now, but acknowledge positional memory exists.
            # In a full implementation, PathPlanner might receive a reference to PositionalMemory.

            # Example check (replace with more robust logic using PositionalMemory if available in scope)
            # if mission_state.is_in_start_section(): # This flag needs setting logic
            if sensor_data.us_data.right_distance > 100: # Placeholder for "gap detected"
                logger.info("PathPlanner (Parking): Parking entrance potentially found. Initiating parking sequence.")
                return NavigationCommand(NavigationAction.EXECUTE_PARKING, 0.0)
            else:
                logger.debug("PathPlanner (Parking): Navigating towards parking area.")
                # Could follow outer wall or re-trace path.
                return NavigationCommand(NavigationAction.FOLLOW_RIGHT_LANE, 0.4) # Slow approach

        elif phase == mission_state.MissionPhase.PARKING:
            logger.debug("PathPlanner: In PARKING phase.")
            # Execute the specific parking maneuver command.
            # The actual sequence is handled by VehicleController.
            logger.info("PathPlanner: Executing PARKING maneuver command.")
            return NavigationCommand(NavigationAction.EXECUTE_PARKING, 0.0)


    def _needs_obstacle_avoidance(self, sensor_data: 'SensorData') -> tuple[bool, float]:
        """
        Basic obstacle avoidance logic based on ultrasonic sensors.
        Args:
            sensor_data: Latest sensor data.
        Returns:
            tuple[bool, float]: (needs_avoidance, turn_direction).
                            turn_direction: -1=left, 1=right, 0=none/straight.
        """
        # Thresholds (these are examples, need tuning)
        front_threshold = 20.0  # cm
        side_threshold = 15.0   # cm

        front_dist = sensor_data.us_data.front_distance
        left_dist = sensor_data.us_data.left_distance
        right_dist = sensor_data.us_data.right_distance
        # back_dist = sensor_data.us_data.back_distance # Might be useful for backing up

        if front_dist < front_threshold and front_dist > 0: # Valid reading and too close
            # Obstacle in front
            if left_dist > right_dist and left_dist > side_threshold:
                # More space on the left
                logger.debug(f"ObstacleAvoidance: Front obstacle. Turning LEFT. (F:{front_dist:.1f}, L:{left_dist:.1f}, R:{right_dist:.1f})")
                return True, -1.0 # Turn left
            elif right_dist > left_dist and right_dist > side_threshold:
                # More space on the right
                logger.debug(f"ObstacleAvoidance: Front obstacle. Turning RIGHT. (F:{front_dist:.1f}, L:{left_dist:.1f}, R:{right_dist:.1f})")
                return True, 1.0 # Turn right
            else:
                # No clear side preference or boxed in
                logger.debug(f"ObstacleAvoidance: Front obstacle. No clear side or boxed in. (F:{front_dist:.1f}, L:{left_dist:.1f}, R:{right_dist:.1f})")
                return True, 0.0 # Go straight/back up?

        # Could also check for side obstacles forcing a turn away
        # e.g., if left_dist < side_threshold and right_dist > left_dist + 5:
        #       return True, 1.0 # Turn right to avoid left wall

        # No immediate front obstacle requiring action
        return False, 0.0


# Example usage (if run as script)
if __name__ == "__main__":
    import utils.logger
    utils.logger.setup_logging()

    # This would normally be integrated into the main application flow
    # and receive real data objects.
    planner = PathPlanner()
    # Mock dependencies would be needed to run this standalone
    print("PathPlanner class defined.")

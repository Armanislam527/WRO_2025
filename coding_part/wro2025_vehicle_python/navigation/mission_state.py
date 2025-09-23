# navigation/mission_state.py

"""Manages the overall mission state of the vehicle."""

import threading
import logging

logger = logging.getLogger(__name__)

# --- Enums (Matching C++ enums where possible) ---

class ChallengeType:
    """Represents the type of challenge."""
    UNKNOWN = 0
    OPEN_CHALLENGE = 1
    OBSTACLE_CHALLENGE = 2

class MissionPhase:
    """Represents the current phase of the mission."""
    PRE_START = 0           # Initial state, waiting for button press
    WAITING_FOR_GO = 1      # Button pressed, waiting for Pi's GO command ack
    DRIVING_LAPS = 2        # Main driving phase (3 laps)
    STOPPING_AT_START = 3   # Open Challenge: Stop in start section after 3 laps
    FINDING_PARKING = 4     # Obstacle Challenge: Navigate to parking lot after 3 laps
    PARKING = 5             # Obstacle Challenge: Execute parking maneuver
    EMERGENCY_STOPPED = 6   # Stopped due to E-Stop or error
    COMPLETED = 7           # Mission finished (vehicle stopped correctly)

class StartPosition:
    """Represents the starting position/direction (simplified)."""
    # This could be more complex (e.g., Section 1, Direction Clockwise)
    # For now, a simple flag or enum might suffice if position is determined dynamically
    UNKNOWN = 0
    IDENTIFIED = 1 # Vehicle has figured out its start position


class MissionState:
    """
    Tracks the overall state of the mission including phase, challenge type,
    lap count, and positional flags.
    Thread-safe using a lock.
    """

    def __init__(self):
        self._lock = threading.RLock() # Reentrant lock

        # Core State
        self._challenge_type = ChallengeType.UNKNOWN
        self._phase = MissionPhase.PRE_START
        self._start_position = StartPosition.UNKNOWN
        self._current_lap = 0 # 0-based (0, 1, 2)
        self._lap_completed = False # Flag for lap completion logic
        self._in_start_section = False # Flag set by lap counter/navigation
        self._near_parking_lot = False # Flag set by navigation (Obstacle)

        logger.debug("MissionState initialized.")

    # --- Getters ---
    def get_challenge_type(self) -> ChallengeType:
        with self._lock:
            return self._challenge_type

    def get_phase(self) -> MissionPhase:
        with self._lock:
            return self._phase

    def get_start_position(self) -> StartPosition:
        with self._lock:
            return self._start_position

    def get_current_lap(self) -> int:
        with self._lock:
            return self._current_lap

    def is_lap_completed(self) -> bool:
        with self._lock:
            return self._lap_completed

    def is_in_start_section(self) -> bool:
        with self._lock:
            return self._in_start_section

    def is_near_parking_lot(self) -> bool:
        with self._lock:
            return self._near_parking_lot

    # --- Setters (Thread-safe) ---
    def set_challenge_type(self, challenge_type: ChallengeType):
        with self._lock:
            old_type = self._challenge_type
            self._challenge_type = challenge_type
            if old_type != challenge_type:
                logger.info(f"Challenge type changed from {old_type} to {challenge_type}")

    def set_phase(self, phase: MissionPhase):
        with self._lock:
            old_phase = self._phase
            self._phase = phase
            if old_phase != phase:
                logger.info(f"Mission phase changed from {old_phase} to {phase}")

    def set_start_position(self, position: StartPosition):
        with self._lock:
            self._start_position = position
            logger.debug(f"Start position set to {position}")

    def set_current_lap(self, lap: int):
        with self._lock:
            self._current_lap = lap
            logger.debug(f"Current lap set to {lap}")

    def set_lap_completed(self, completed: bool):
        with self._lock:
            self._lap_completed = completed
            logger.debug(f"Lap completed flag set to {completed}")

    def set_in_start_section(self, in_start: bool):
        with self._lock:
            self._in_start_section = in_start
            logger.debug(f"In start section flag set to {in_start}")

    def set_near_parking_lot(self, near: bool):
        with self._lock:
            self._near_parking_lot = near
            logger.debug(f"Near parking lot flag set to {near}")

    # --- Utility Methods ---
    def is_obstacle_challenge(self) -> bool:
        return self.get_challenge_type() == ChallengeType.OBSTACLE_CHALLENGE

    def is_open_challenge(self) -> bool:
        return self.get_challenge_type() == ChallengeType.OPEN_CHALLENGE

    def is_driving_phase(self) -> bool:
        """Check if the vehicle is in an active driving phase."""
        phase = self.get_phase()
        return (
            phase == MissionPhase.DRIVING_LAPS or
            phase == MissionPhase.FINDING_PARKING or
            phase == MissionPhase.PARKING or
            phase == MissionPhase.STOPPING_AT_START
        )

    def acknowledge_lap_completion(self):
        """Acknowledge that a lap completion has been processed."""
        with self._lock:
            self._lap_completed = False
            logger.debug("Lap completion acknowledged.")

    def __str__(self):
        with self._lock:
            return (f"MissionState(Phase: {self._phase}, Challenge: {self._challenge_type}, "
                    f"Lap: {self._current_lap}, InStart: {self._in_start_section}, "
                    f"NearPark: {self._near_parking_lot})")

# Example usage (if run as script)
if __name__ == "__main__":
    import utils.logger
    utils.logger.setup_logging()
    
    ms = MissionState()
    print(ms)
    ms.set_challenge_type(ChallengeType.OPEN_CHALLENGE)
    logger.info(f"[Main] Explicitly set challenge type to Open. Current state: {ms}")
    ms.set_phase(MissionPhase.DRIVING_LAPS)
    ms.set_current_lap(1)
    print(ms)
    print(f"Is Driving Phase? {ms.is_driving_phase()}")
    print(f"Is Open Challenge? {ms.is_open_challenge()}")

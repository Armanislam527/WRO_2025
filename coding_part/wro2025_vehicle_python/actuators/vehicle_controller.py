# actuators/vehicle_controller.py
"""Controls the actuators (motors, servos) via commands sent to the Nano."""

import logging
import time
from enum import IntEnum
from typing import TYPE_CHECKING, Optional
from control.navigation_command import NavigationAction
if TYPE_CHECKING:
    from communication.serial_handler import SerialHandler
    from control.navigation_command import NavigationCommand, NavigationAction
    from sensors.sensor_data import SensorData

logger = logging.getLogger(__name__)

class ParkingState(IntEnum):
    """Represents the state within the parking sequence."""
    APPROACHING_SPOT = 0
    REVERSING_INTO_SPOT = 1
    ADJUSTING_POSITION = 2
    PARKING_COMPLETE = 3
    PARKING_FAILED = 4

class VehicleController:
    """
    Translates high-level NavigationCommands into low-level actuator commands
    sent via the SerialHandler. Manages complex maneuvers like parking.
    """

    def __init__(self, serial_handler: 'SerialHandler'):
        if not serial_handler:
            raise ValueError("VehicleController requires a valid SerialHandler.")
        self.serial_handler = serial_handler

        # Parking state management
        self.parking_active = False
        self.parking_state: ParkingState = ParkingState.APPROACHING_SPOT
        self.parking_step_counter = 0
        self.parking_start_time = 0.0

        # Last sent commands for potential logging or smoothing
        self._last_motor_speed = 0.0
        self._last_servo_angle = 90 # Assume 90 is center

        logger.info("VehicleController initialized.")

    def execute_command(self, command: 'NavigationCommand'):
        """
        Execute a NavigationCommand by sending appropriate motor/servo commands.
        This is the primary interface for the main loop.
        """
        if not command:
            logger.warning("VehicleController: Received None command. Ignoring.")
            return

        action = command.action
        desired_speed = command.desired_speed

        # Handle complex maneuvers first
        if self.parking_active:
            self._execute_parking_step()
            return # Parking takes precedence

        # Map NavigationAction to motor/servo commands
        if action == NavigationAction.GO_STRAIGHT:
            # Speed determines magnitude, servo centered
            motor_speed = self._map_normalized_speed(desired_speed)
            servo_angle = 90 # Straight
            self._send_motor_command(motor_speed)
            self._send_servo_command(servo_angle)

        elif action in (NavigationAction.TURN_LEFT, NavigationAction.FOLLOW_LEFT_LANE):
            motor_speed = self._map_normalized_speed(desired_speed)
            # Negative angle for left turn (assuming 0 is full left, 180 is full right)
            servo_angle = max(0, 90 - 30) # Adjust turn angle as needed
            self._send_motor_command(motor_speed)
            self._send_servo_command(servo_angle)

        elif action in (NavigationAction.TURN_RIGHT, NavigationAction.FOLLOW_RIGHT_LANE):
            motor_speed = self._map_normalized_speed(desired_speed)
            # Positive angle for right turn
            servo_angle = min(180, 90 + 30) # Adjust turn angle as needed
            self._send_motor_command(motor_speed)
            self._send_servo_command(servo_angle)

        elif action == NavigationAction.STOP_IN_START:
            # Gradual stop or immediate stop?
            self._send_motor_command(0)
            self._send_servo_command(90) # Center steering
            logger.info("VehicleController: Executing STOP_IN_START.")

        elif action == NavigationAction.EXECUTE_PARKING:
            # Initiate the parking sequence
            self.start_parking_sequence()

        elif action == NavigationAction.EMERGENCY_STOP:
            self.emergency_stop()
        else:
            logger.warning(f"VehicleController: Unknown action {action}. Stopping.")
            self._send_motor_command(0)
            self._send_servo_command(90)

    def _map_normalized_speed(self, normalized_speed: float) -> int:
        """
        Map a normalized speed (0.0 to 1.0) to an integer motor command (-255 to 255).
        Negative values indicate reverse.
        """
        # Clamp input
        clamped_speed = max(-1.0, min(1.0, normalized_speed))
        # Map to motor range (assuming -255 to 255)
        mapped_speed = int(clamped_speed * 255)
        # Clamp output to motor range if library expects it (pyserial doesn't care)
        return max(-255, min(255, mapped_speed))

    def _send_motor_command(self, speed: int):
        """Send motor speed command via serial handler."""
        if speed != self._last_motor_speed:
            logger.debug(f"VehicleController: Sending motor speed {speed}")
            self.serial_handler.sendMotorSpeedCommand(speed)
            self._last_motor_speed = speed

    def _send_servo_command(self, angle: int):
        """Send servo angle command via serial handler."""
        # Clamp angle to typical servo range if not already
        clamped_angle = max(0, min(180, angle))
        if clamped_angle != self._last_servo_angle:
            logger.debug(f"VehicleController: Sending servo angle {clamped_angle}")
            self.serial_handler.sendServoAngleCommand(clamped_angle)
            self._last_servo_angle = clamped_angle

    # --- Parking Sequence Logic ---

    def start_parking_sequence(self):
        """Initiate the parallel parking sequence."""
        if self.parking_active:
            logger.warning("VehicleController: Parking already active. Cannot start new sequence.")
            return

        logger.info("VehicleController: Starting parallel parking sequence.")
        self.parking_active = True
        self.parking_state = ParkingState.APPROACHING_SPOT
        self.parking_step_counter = 0
        self.parking_start_time = time.time()
        # Reset any previous parking state variables if needed

    def _execute_parking_step(self):
        """Execute the current step of the parking sequence."""
        if not self.parking_active:
            return

        self.parking_step_counter += 1
        current_time = time.time()
        elapsed_time = current_time - self.parking_start_time

        # --- Simplified Parking State Machine ---
        # This is a basic placeholder, mirroring the C++ logic's structure.
        # A real implementation would use encoders, IMU, or precise timing/distance.
        try:
            if self.parking_state == ParkingState.APPROACHING_SPOT:
                # 1. Approach the spot slowly, aligning the rear axle
                self._send_motor_command(self._map_normalized_speed(0.3)) # Slow forward
                self._send_servo_command(90) # Straight
                logger.debug(f"Parking Step {self.parking_step_counter}: APPROACHING_SPOT")
                # Placeholder condition to move to next state
                if self.parking_step_counter > 50: # Or based on time/distance/IMU
                    logger.info("Parking: Transitioning to REVERSING_INTO_SPOT")
                    self.parking_state = ParkingState.REVERSING_INTO_SPOT
                    self.parking_step_counter = 0

            elif self.parking_state == ParkingState.REVERSING_INTO_SPOT:
                # 2. Reverse while turning steering wheel
                self._send_motor_command(self._map_normalized_speed(-0.4)) # Slow reverse
                self._send_servo_command(45) # Turn wheels (e.g., full left)
                logger.debug(f"Parking Step {self.parking_step_counter}: REVERSING_INTO_SPOT")
                # Placeholder condition
                if self.parking_step_counter > 70:
                    logger.info("Parking: Transitioning to ADJUSTING_POSITION")
                    self.parking_state = ParkingState.ADJUSTING_POSITION
                    self.parking_step_counter = 0

            elif self.parking_state == ParkingState.ADJUSTING_POSITION:
                # 3. Adjust position (e.g., straighten wheels, move forward/back)
                self._send_motor_command(self._map_normalized_speed(0.3)) # Slow forward
                self._send_servo_command(135) # Turn wheels (e.g., full right)
                logger.debug(f"Parking Step {self.parking_step_counter}: ADJUSTING_POSITION")
                # Placeholder condition
                if self.parking_step_counter > 30:
                    logger.info("Parking: Transitioning to PARKING_COMPLETE")
                    self.parking_state = ParkingState.PARKING_COMPLETE
                    self.parking_step_counter = 0

            elif self.parking_state == ParkingState.PARKING_COMPLETE:
                # 4. Finalize parking
                self._send_motor_command(0) # Stop
                self._send_servo_command(90) # Center
                logger.info("VehicleController: PARKING SEQUENCE COMPLETED.")
                self.parking_active = False # End the sequence
                # Potentially update MissionState phase to COMPLETED here or in main loop

            elif self.parking_state == ParkingState.PARKING_FAILED:
                # Handle failure (e.g., timeout, obstacle detected)
                logger.error("VehicleController: PARKING SEQUENCE FAILED.")
                self._send_motor_command(0)
                self._send_servo_command(90)
                self.parking_active = False

            # Optional: Add a timeout mechanism
            if elapsed_time > 30.0: # e.g., 30 seconds max
                logger.error("VehicleController: Parking timeout. Aborting.")
                self.parking_state = ParkingState.PARKING_FAILED
                self._execute_parking_step() # Re-enter to handle FAILED state

        except Exception as e:
            logger.error(f"VehicleController: Error during parking step execution: {e}")
            self.parking_state = ParkingState.PARKING_FAILED
            self._execute_parking_step() # Attempt to handle error state

    def is_maneuver_active(self) -> bool:
        """Check if a complex maneuver (like parking) is currently active."""
        return self.parking_active # Could extend to other maneuvers

    def emergency_stop(self):
        """Immediately send stop commands and halt maneuvers."""
        logger.critical("VehicleController: EMERGENCY STOP INITIATED!")
        self.parking_active = False # Cancel any active maneuver
        self._send_motor_command(0)
        self._send_servo_command(90)
        self.serial_handler.sendEmergencyStop() # Send dedicated E-Stop command


# Example usage (if run as script - needs mocks)
if __name__ == "__main__":
    # This would normally be integrated into the main application flow
    print("VehicleController class defined.")

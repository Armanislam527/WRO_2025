"""
State machine for WRO 2025 competition phases
Manages transitions between competition states and coordinates all subsystems
"""

import time
from enum import Enum, auto
from typing import Dict, Any, Callable, Optional
from dataclasses import dataclass
from config.vehicle_config import RULES
from core.shared_memory import shared_memory

class CompetitionState(Enum):
    """All possible states of the competition vehicle"""
    SYSTEM_STARTUP = auto()
    CALIBRATION = auto()
    AWAITING_START = auto()
    OPEN_CHALLENGE = auto()
    OBSTACLE_CHALLENGE = auto()
    PARKING_MANEUVER = auto()
    MISSION_COMPLETE = auto()
    EMERGENCY_STOP = auto()
    SYSTEM_SHUTDOWN = auto()

@dataclass
class StateTransition:
    """Definition of a state transition"""
    from_state: CompetitionState
    to_state: CompetitionState
    condition: Callable[[], bool]
    description: str

class CompetitionStateMachine:
    """
    Main state machine for WRO 2025 competition
    Coordinates all subsystems based on current competition phase
    """
    
    def __init__(self):
        self.current_state = CompetitionState.SYSTEM_STARTUP
        self.previous_state = None
        self.state_start_time = time.time()
        self.transitions: Dict[CompetitionState, list] = {}
        self.state_handlers = {}
        
        # Competition parameters
        self.challenge_type = "OBSTACLE"  # Determined at runtime
        self.driving_direction = 1  # 1 = clockwise, -1 = counter-clockwise
        self.lap_count = 0
        self.score_estimate = 0
        
        # Initialize state machine
        self._setup_transitions()
        self._setup_state_handlers()
        
        print("🎯 Competition state machine initialized")
    
    def _setup_transitions(self):
        """Define all possible state transitions"""
        self.transitions = {
            CompetitionState.SYSTEM_STARTUP: [
                StateTransition(
                    CompetitionState.SYSTEM_STARTUP,
                    CompetitionState.CALIBRATION,
                    self._system_ready,
                    "All systems initialized and ready"
                )
            ],
            CompetitionState.CALIBRATION: [
                StateTransition(
                    CompetitionState.CALIBRATION,
                    CompetitionState.AWAITING_START,
                    self._calibration_complete,
                    "Calibration completed successfully"
                )
            ],
            CompetitionState.AWAITING_START: [
                StateTransition(
                    CompetitionState.AWAITING_START,
                    CompetitionState.OPEN_CHALLENGE,
                    self._open_challenge_started,
                    "Open challenge start signal received"
                ),
                StateTransition(
                    CompetitionState.AWAITING_START,
                    CompetitionState.OBSTACLE_CHALLENGE,
                    self._obstacle_challenge_started,
                    "Obstacle challenge start signal received"
                )
            ],
            CompetitionState.OPEN_CHALLENGE: [
                StateTransition(
                    CompetitionState.OPEN_CHALLENGE,
                    CompetitionState.MISSION_COMPLETE,
                    self._open_challenge_complete,
                    "Open challenge completed (3 laps)"
                ),
                StateTransition(
                    CompetitionState.OPEN_CHALLENGE,
                    CompetitionState.EMERGENCY_STOP,
                    self._emergency_condition,
                    "Emergency stop required"
                )
            ],
            CompetitionState.OBSTACLE_CHALLENGE: [
                StateTransition(
                    CompetitionState.OBSTACLE_CHALLENGE,
                    CompetitionState.PARKING_MANEUVER,
                    self._obstacle_laps_complete,
                    "3 laps completed, starting parking"
                ),
                StateTransition(
                    CompetitionState.OBSTACLE_CHALLENGE,
                    CompetitionState.EMERGENCY_STOP,
                    self._emergency_condition,
                    "Emergency stop required"
                )
            ],
            CompetitionState.PARKING_MANEUVER: [
                StateTransition(
                    CompetitionState.PARKING_MANEUVER,
                    CompetitionState.MISSION_COMPLETE,
                    self._parking_complete,
                    "Parking maneuver completed"
                ),
                StateTransition(
                    CompetitionState.PARKING_MANEUVER,
                    CompetitionState.EMERGENCY_STOP,
                    self._emergency_condition,
                    "Emergency stop required"
                )
            ],
            CompetitionState.MISSION_COMPLETE: [
                StateTransition(
                    CompetitionState.MISSION_COMPLETE,
                    CompetitionState.SYSTEM_SHUTDOWN,
                    self._shutdown_ready,
                    "Ready for system shutdown"
                )
            ],
            CompetitionState.EMERGENCY_STOP: [
                StateTransition(
                    CompetitionState.EMERGENCY_STOP,
                    CompetitionState.SYSTEM_SHUTDOWN,
                    self._emergency_resolved,
                    "Emergency condition resolved"
                )
            ]
        }
    
    def _setup_state_handlers(self):
        """Setup handlers for each state"""
        self.state_handlers = {
            CompetitionState.SYSTEM_STARTUP: self._handle_system_startup,
            CompetitionState.CALIBRATION: self._handle_calibration,
            CompetitionState.AWAITING_START: self._handle_awaiting_start,
            CompetitionState.OPEN_CHALLENGE: self._handle_open_challenge,
            CompetitionState.OBSTACLE_CHALLENGE: self._handle_obstacle_challenge,
            CompetitionState.PARKING_MANEUVER: self._handle_parking_maneuver,
            CompetitionState.MISSION_COMPLETE: self._handle_mission_complete,
            CompetitionState.EMERGENCY_STOP: self._handle_emergency_stop,
            CompetitionState.SYSTEM_SHUTDOWN: self._handle_system_shutdown
        }
    
    def update(self) -> CompetitionState:
        """Update state machine - called regularly from main loop"""
        # Check for transitions from current state
        if self.current_state in self.transitions:
            for transition in self.transitions[self.current_state]:
                if transition.condition():
                    self._transition_to(transition.to_state, transition.description)
                    break
        
        # Execute current state handler
        if self.current_state in self.state_handlers:
            self.state_handlers[self.current_state]()
        
        return self.current_state
    
    def _transition_to(self, new_state: CompetitionState, reason: str):
        """Transition to a new state"""
        if self.current_state != new_state:
            print(f"🔄 State transition: {self.current_state.name} → {new_state.name}")
            print(f"   Reason: {reason}")
            
            self.previous_state = self.current_state
            self.current_state = new_state
            self.state_start_time = time.time()
            
            # Execute transition actions
            self._on_state_exit(self.previous_state)
            self._on_state_enter(new_state)
    
    def _on_state_enter(self, state: CompetitionState):
        """Actions to perform when entering a state"""
        if state == CompetitionState.OBSTACLE_CHALLENGE:
            print("🚦 Obstacle challenge started - watching for traffic signs")
        elif state == CompetitionState.PARKING_MANEUVER:
            print("🅿️ Parking maneuver initiated - enabling parking detection")
            # Enable parking detection in vision system
            from vision.vision_integrator import vision_integrator
            vision_integrator.enable_parking_detection(True)
        elif state == CompetitionState.MISSION_COMPLETE:
            print("🎉 Mission completed! Estimating final score...")
            self._calculate_final_score()
    
    def _on_state_exit(self, state: CompetitionState):
        """Actions to perform when exiting a state"""
        if state == CompetitionState.PARKING_MANEUVER:
            # Disable parking detection to save CPU
            from vision.vision_integrator import vision_integrator
            vision_integrator.enable_parking_detection(False)
    
    def _handle_system_startup(self):
        """Handle system startup state"""
        # Check if all systems are ready
        if (shared_memory.nano_connected.value and 
            shared_memory.camera_ready.value and
            shared_memory.system_ready.value):
            print("✅ All systems ready for calibration")
    
    def _handle_calibration(self):
        """Handle calibration state"""
        # Perform sensor calibration
        elapsed = time.time() - self.state_start_time
        if elapsed > 5.0:  # Simulate 5-second calibration
            print("✅ Sensor calibration completed")
    
    def _handle_awaiting_start(self):
        """Handle awaiting start signal"""
        # Wait for external start signal (from judge or button)
        pass
    
    def _handle_open_challenge(self):
        """Handle open challenge execution"""
        # Update lap counting and navigation
        self._update_lap_counting()
        
        # Control system will handle lane following automatically
        pass
    
    def _handle_obstacle_challenge(self):
        """Handle obstacle challenge execution"""
        # Update lap counting
        self._update_lap_counting()
        
        # Monitor traffic sign compliance
        self._monitor_traffic_signs()
    
    def _handle_parking_maneuver(self):
        """Handle parking maneuver execution"""
        # Parking is handled by the control system
        # We just monitor completion here
        pass
    
    def _handle_mission_complete(self):
        """Handle mission completion"""
        # Vehicle should be stopped
        control_state = shared_memory.get_control_state()
        control_state.target_speed = 0
        control_state.operation_mode = "IDLE"
        shared_memory.update_control_state(control_state)
        
        # Wait for shutdown signal
        if time.time() - self.state_start_time > 10.0:  # 10 seconds after completion
            print("⏳ Mission complete - awaiting shutdown signal")
    
    def _handle_emergency_stop(self):
        """Handle emergency stop state"""
        # Ensure vehicle is stopped
        control_state = shared_memory.get_control_state()
        control_state.emergency_stop = True
        control_state.target_speed = 0
        shared_memory.update_control_state(control_state)
        
        # Send emergency stop to Nano
        from communication.serial_handler import serial_handler
        serial_handler.send_emergency_stop()
        
        print("🚨 EMERGENCY STOP ACTIVATED - Vehicle halted")
    
    def _handle_system_shutdown(self):
        """Handle system shutdown"""
        # Cleanup operations will be handled by process manager
        pass
    
    # Transition conditions
    def _system_ready(self) -> bool:
        return (shared_memory.nano_connected.value and 
                shared_memory.camera_ready.value)
    
    def _calibration_complete(self) -> bool:
        return time.time() - self.state_start_time > 5.0  # 5-second calibration
    
    def _open_challenge_started(self) -> bool:
        # This would be triggered by external signal
        return False  # Will be set externally
    
    def _obstacle_challenge_started(self) -> bool:
        # This would be triggered by external signal
        return False  # Will be set externally
    
    def _open_challenge_complete(self) -> bool:
        # Check if 3 laps are completed
        return self.lap_count >= 3
    
    def _obstacle_laps_complete(self) -> bool:
        # Check if 3 laps are completed for obstacle challenge
        return self.lap_count >= 3
    
    def _parking_complete(self) -> bool:
        # Check if parking is completed
        vision_data = shared_memory.get_vision_data()
        return vision_data.parking_detected and self.lap_count >= 3
    
    def _emergency_condition(self) -> bool:
        # Check for emergency conditions
        control_state = shared_memory.get_control_state()
        return (control_state.emergency_stop or 
                not shared_memory.nano_connected.value)
    
    def _emergency_resolved(self) -> bool:
        # Check if emergency is resolved
        return False  # Typically requires manual intervention
    
    def _shutdown_ready(self) -> bool:
        # Ready for shutdown after mission completion
        return time.time() - self.state_start_time > 5.0
    
    def _update_lap_counting(self):
        """Update lap counting based on section progress"""
        # Simplified lap counting - would use sensor fusion in real implementation
        elapsed = time.time() - self.state_start_time
        estimated_lap_time = 30.0  # 30 seconds per lap (estimate)
        
        new_lap_count = min(3, int(elapsed / estimated_lap_time))
        if new_lap_count > self.lap_count:
            self.lap_count = new_lap_count
            print(f"⏱️ Lap {self.lap_count} completed")
    
    def _monitor_traffic_signs(self):
        """Monitor traffic sign compliance during obstacle challenge"""
        vision_data = shared_memory.get_vision_data()
        if vision_data.traffic_sign_detected:
            print(f"🚦 Traffic sign detected: {vision_data.sign_color}")
    
    def _calculate_final_score(self):
        """Calculate estimated final score based on performance"""
        base_score = self.lap_count * 8  # 8 points per lap
        
        if self.challenge_type == "OBSTACLE" and self.lap_count >= 3:
            base_score += 10  # Bonus for completing 3 laps with signs
            base_score += 15  # Parking bonus (assuming success)
        
        self.score_estimate = base_score
        print(f"📊 Estimated score: {self.score_estimate} points")
    
    def get_state_info(self) -> Dict[str, Any]:
        """Get current state information"""
        return {
            "current_state": self.current_state.name,
            "previous_state": self.previous_state.name if self.previous_state else None,
            "state_duration": time.time() - self.state_start_time,
            "lap_count": self.lap_count,
            "challenge_type": self.challenge_type,
            "driving_direction": "clockwise" if self.driving_direction == 1 else "counter-clockwise",
            "score_estimate": self.score_estimate
        }
    
    def trigger_challenge_start(self, challenge_type: str, direction: int = 1):
        """Trigger challenge start (called externally)"""
        self.challenge_type = challenge_type
        self.driving_direction = direction
        
        if challenge_type == "OPEN":
            self._transition_to(CompetitionState.OPEN_CHALLENGE, "Open challenge started externally")
        else:
            self._transition_to(CompetitionState.OBSTACLE_CHALLENGE, "Obstacle challenge started externally")

# Global state machine instance
state_machine = CompetitionStateMachine()
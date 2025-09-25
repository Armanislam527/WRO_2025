"""
Navigation controller for WRO 2025 competition
Implements competition-specific logic for Open Challenge and Obstacle Challenge
Optimized for Pi Zero 2W with efficient decision making
"""

import time
import math
from typing import Dict, Tuple, Optional
from dataclasses import dataclass
from config.vehicle_config import RULES, VEHICLE, PERF
from core.shared_memory import SensorData, VisionData

@dataclass
class NavigationState:
    """Current navigation state for decision making"""
    current_lap: int = 0
    sections_completed: int = 0
    last_section_time: float = 0.0
    lap_start_time: float = 0.0
    traffic_signs_passed: int = 0
    signs_moved: int = 0
    parking_attempted: bool = False
    mission_complete: bool = False

class NavigationController:
    """
    Main navigation controller for WRO 2025 competition
    Implements state machines for both Open and Obstacle challenges
    """
    
    def __init__(self):
        self.state = NavigationState()
        self.challenge_type = "OBSTACLE"  # or "OPEN" - determined at runtime
        self.driving_direction = 1  # 1 for clockwise, -1 for counter-clockwise
        
        # Section tracking (8 sections per lap: 4 straight, 4 corners)
        self.section_entries = [0] * 8
        self.current_section = 0
        
        # Performance metrics
        self.start_time = 0.0
        self.decision_count = 0
        
    def initialize_challenge(self, challenge_type: str, driving_direction: int) -> None:
        """Initialize navigation for specific challenge type"""
        self.challenge_type = challenge_type
        self.driving_direction = driving_direction
        self.state = NavigationState()
        self.state.lap_start_time = time.time()
        self.start_time = time.time()
        
        print(f"🧭 Navigation initialized: {challenge_type} challenge, "
              f"direction: {'clockwise' if driving_direction == 1 else 'counter-clockwise'}")
    
    def update_navigation(self, sensor_data: SensorData, vision_data: VisionData) -> str:
        """
        Main navigation update - returns recommended operation mode
        This is the core decision-making function
        """
        self.decision_count += 1
        
        # Update section tracking based on vehicle movement
        self._update_section_tracking(sensor_data, vision_data)
        
        # Determine current operation mode based on challenge type
        if self.challenge_type == "OPEN":
            return self._open_challenge_navigation(sensor_data, vision_data)
        else:  # OBSTACLE challenge
            return self._obstacle_challenge_navigation(sensor_data, vision_data)
    
    def _open_challenge_navigation(self, sensor_data: SensorData, vision_data: VisionData) -> str:
        """Navigation logic for Open Challenge (just lane following)"""
        if self.state.mission_complete:
            return "IDLE"
        
        # Check if we've completed 3 laps
        if self.state.current_lap >= 3:
            # Try to stop in the finish section for bonus points
            if self._in_finish_section():
                self.state.mission_complete = True
                print("🎉 Open Challenge completed! Stopping in finish section.")
                return "IDLE"
            else:
                # Continue to finish section
                return "LANE_FOLLOWING"
        
        # Normal lane following for open challenge
        return "LANE_FOLLOWING"
    
    def _obstacle_challenge_navigation(self, sensor_data: SensorData, vision_data: VisionData) -> str:
        """Navigation logic for Obstacle Challenge (signs + parking)"""
        if self.state.mission_complete:
            return "IDLE"
        
        # Phase 1: Complete 3 laps with traffic signs
        if self.state.current_lap < 3:
            # Check for traffic signs
            if vision_data.traffic_sign_detected and not self._recently_passed_sign():
                return "SIGN_AVOIDANCE"
            else:
                return "LANE_FOLLOWING"
        
        # Phase 2: After 3 laps, look for parking
        elif not self.state.parking_attempted:
            if vision_data.parking_detected:
                print("🅿️ Parking spot detected - beginning parking maneuver")
                self.state.parking_attempted = True
                return "PARKING"
            else:
                # Continue driving to find parking spot
                return "LANE_FOLLOWING"
        
        # Phase 3: Parking completed
        else:
            self.state.mission_complete = True
            print("🎉 Obstacle Challenge completed!")
            return "IDLE"
    
    def _update_section_tracking(self, sensor_data: SensorData, vision_data: VisionData) -> None:
        """Track which section of the track we're in for lap counting"""
        # This is a simplified implementation - would use sensor fusion in real system
        # For now, we'll use time-based section estimation
        
        current_time = time.time()
        section_duration = 3.0  # Estimated seconds per section
        
        # Check if we've entered a new section
        if current_time - self.state.last_section_time > section_duration:
            self.state.sections_completed += 1
            self.state.last_section_time = current_time
            
            # Update lap count (8 sections per lap)
            if self.state.sections_completed % 8 == 0:
                self.state.current_lap += 1
                lap_time = current_time - self.state.lap_start_time
                self.state.lap_start_time = current_time
                
                print(f"⏱️ Lap {self.state.current_lap} completed in {lap_time:.1f}s")
                
                # Check for challenge completion
                if self.state.current_lap >= 3 and self.challenge_type == "OPEN":
                    print("📋 Open Challenge: 3 laps completed, looking for finish section")
    
    def _in_finish_section(self) -> bool:
        """Check if we're in the finish section (simplified)"""
        # In real implementation, this would use precise position tracking
        # For now, we'll use a simple time-based approximation
        return self.state.sections_completed % 8 == 0  # Start section is finish section
    
    def _recently_passed_sign(self) -> bool:
        """Check if we recently passed a sign to avoid duplicate detections"""
        current_time = time.time()
        return current_time - self.state.last_section_time < 2.0  # 2 second cooldown
    
    def get_performance_metrics(self) -> Dict[str, float]:
        """Get navigation performance metrics"""
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        return {
            "decision_rate": self.decision_count / elapsed if elapsed > 0 else 0,
            "current_lap": self.state.current_lap,
            "sections_completed": self.state.sections_completed,
            "elapsed_time": elapsed,
            "signs_passed": self.state.traffic_signs_passed,
            "mission_complete": self.state.mission_complete
        }
    
    def get_estimated_score(self) -> Tuple[int, str]:
        """Estimate current score based on WRO 2025 scoring rules"""
        base_score = 0
        details = []
        
        if self.challenge_type == "OPEN":
            # Open Challenge scoring (max 30 points)
            base_score = min(self.state.current_lap, 3) * 8  # 8 points per lap
            if self.state.mission_complete and self._in_finish_section():
                base_score += 3  # Bonus for stopping in finish section
                details.append("Finish section bonus: +3")
                
        else:  # OBSTACLE challenge
            # Obstacle Challenge scoring (max 62 points)
            base_score = min(self.state.current_lap, 3) * 8  # 8 points per lap
            
            if self.state.current_lap >= 3:
                if self.state.signs_moved == 0:
                    base_score += 10  # No signs moved bonus
                    details.append("No signs moved: +10")
                else:
                    base_score += 8  # Signs moved but completed
                    details.append(f"Signs moved: {self.state.signs_moved}, bonus: +8")
                
                if self.state.parking_attempted:
                    base_score += 7  # Partial parking
                    details.append("Parking attempted: +7")
                    if self.state.mission_complete:
                        base_score += 8  # Successful parking completion
                        details.append("Parking completed: +8")
        
        return base_score, "; ".join(details)

# Global navigation controller instance
navigation_controller = NavigationController()
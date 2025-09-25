"""
Motor controller for WRO 2025 vehicle
Implements speed control algorithms with safety limits
Optimized for smooth acceleration and competition performance
"""

import math
from typing import Dict
from dataclasses import dataclass
from config.vehicle_config import VEHICLE, SAFETY_CONF

@dataclass
class SpeedProfile:
    """Speed control parameters for different operational modes"""
    max_speed: float = 80.0  # Percentage of max motor power
    acceleration: float = 5.0  # Speed change per second (%/s)
    deceleration: float = 10.0  # Quicker deceleration for safety
    curve_slowdown: float = 0.7  Speed reduction factor in curves

class MotorController:
    """
    Intelligent motor controller with adaptive speed control
    Maintains optimal speed for competition performance and safety
    """
    
    def __init__(self):
        self.current_speed = 0.0
        self.target_speed = 0.0
        self.last_update_time = 0.0
        
        # Speed profiles for different operational modes
        self.speed_profiles = {
            "IDLE": SpeedProfile(0, 20, 20, 1.0),
            "LANE_FOLLOWING": SpeedProfile(80, 5, 10, 0.7),
            "SIGN_AVOIDANCE": SpeedProfile(50, 3, 15, 0.9),
            "PARKING": SpeedProfile(30, 2, 20, 1.0),
            "EMERGENCY": SpeedProfile(0, 50, 50, 1.0)
        }
        
        # Performance tracking
        self.distance_estimated = 0.0  # Estimated distance traveled (meters)
        
    def calculate_speed(self, lane_confidence: float, front_distance: float) -> float:
        """
        Calculate optimal speed for lane following mode
        considering confidence and obstacle distance
        """
        profile = self.speed_profiles["LANE_FOLLOWING"]
        
        # Base speed based on lane confidence
        confidence_factor = min(1.0, lane_confidence * 1.2)  # Boost confidence slightly
        base_speed = profile.max_speed * confidence_factor
        
        # Reduce speed based on front obstacle distance
        obstacle_factor = self._calculate_obstacle_factor(front_distance)
        safe_speed = base_speed * obstacle_factor
        
        self.target_speed = safe_speed
        return self._apply_acceleration_limits()
    
    def calculate_sign_approach_speed(self, front_distance: float) -> float:
        """Calculate speed when approaching traffic signs"""
        profile = self.speed_profiles["SIGN_AVOIDANCE"]
        
        # Slower approach when close to signs
        if front_distance < 50:  # 50cm from sign
            approach_factor = front_distance / 50.0
            safe_speed = profile.max_speed * approach_factor
        else:
            safe_speed = profile.max_speed
        
        self.target_speed = safe_speed
        return self._apply_acceleration_limits()
    
    def calculate_parking_speed(self, front_distance: float, back_distance: float) -> float:
        """Calculate speed during parking maneuver"""
        profile = self.speed_profiles["PARKING"]
        
        # Very slow parking speeds with distance-based control
        min_distance = min(front_distance, back_distance)
        if min_distance < 10:  # 10cm minimum distance
            parking_speed = 0.0  # Stop if too close
        elif min_distance < 30:  # 30cm slow approach
            parking_speed = profile.max_speed * 0.3
        else:
            parking_speed = profile.max_speed
        
        self.target_speed = parking_speed
        return self._apply_acceleration_limits()
    
    def emergency_stop(self) -> float:
        """Execute emergency stop procedure"""
        profile = self.speed_profiles["EMERGENCY"]
        self.target_speed = profile.max_speed  # 0%
        return self._apply_acceleration_limits()
    
    def _calculate_obstacle_factor(self, distance: float) -> float:
        """Calculate speed reduction factor based on obstacle distance"""
        if distance > 200:  # No reduction beyond 2 meters
            return 1.0
        elif distance > 100:  # Gradual reduction from 1-2 meters
            return 0.5 + (distance - 100) / 200
        elif distance > 50:   # More aggressive reduction from 0.5-1 meter
            return 0.3 + (distance - 50) / 250
        elif distance > 20:   # Strong reduction from 20-50cm
            return 0.1 + (distance - 20) / 150
        else:                 # Stop if closer than 20cm
            return 0.0
    
    def _apply_acceleration_limits(self) -> float:
        """Apply acceleration/deceleration limits for smooth speed changes"""
        import time
        current_time = time.time()
        
        if self.last_update_time == 0:
            self.last_update_time = current_time
            self.current_speed = self.target_speed
            return self.current_speed
        
        time_delta = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # Determine appropriate acceleration profile
        profile = self.speed_profiles["LANE_FOLLOWING"]  # Default profile
        if abs(self.target_speed) < abs(self.current_speed):
            # Decelerating
            max_change = profile.deceleration * time_delta
        else:
            # Accelerating
            max_change = profile.acceleration * time_delta
        
        # Apply acceleration limit
        speed_diff = self.target_speed - self.current_speed
        if abs(speed_diff) > max_change:
            if speed_diff > 0:
                self.current_speed += max_change
            else:
                self.current_speed -= max_change
        else:
            self.current_speed = self.target_speed
        
        # Update estimated distance (simplified)
        speed_mps = (self.current_speed / 100.0) * (VEHICLE.MAX_SPEED_MMS / 1000.0)  # m/s
        self.distance_estimated += speed_mps * time_delta
        
        return self.current_speed
    
    def get_motor_status(self) -> Dict[str, float]:
        """Get current motor controller status"""
        return {
            "current_speed": self.current_speed,
            "target_speed": self.target_speed,
            "distance_estimated": self.distance_estimated,
            "acceleration_limit": self.speed_profiles["LANE_FOLLOWING"].acceleration
        }

# Global motor controller instance
motor_controller = MotorController()
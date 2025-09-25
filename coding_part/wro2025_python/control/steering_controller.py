"""
Steering controller for WRO 2025 vehicle
Implements precise steering control for lane following and obstacle avoidance
Uses PID control for smooth and accurate steering
"""

import math
from typing import Dict, Tuple
from dataclasses import dataclass
from config.vehicle_config import VEHICLE

@dataclass
class PIDCoefficients:
    """PID control coefficients for different steering scenarios"""
    kp: float = 0.8   # Proportional gain
    ki: float = 0.01  # Integral gain  
    kd: float = 0.2   # Derivative gain
    max_integral: float = 10.0  # Anti-windup limit

class SteeringController:
    """
    Advanced steering controller with PID control
    Handles lane following, sign avoidance, and parking maneuvers
    """
    
    def __init__(self):
        # PID controllers for different scenarios
        self.lane_pid = PIDCoefficients(0.8, 0.01, 0.2)
        self.sign_pid = PIDCoefficients(1.2, 0.02, 0.3)  # More aggressive for signs
        self.parking_pid = PIDCoefficients(0.5, 0.005, 0.1)  # Slower for parking
        
        # Control state
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_update_time = 0.0
        
        # Steering limits (degrees from center)
        self.max_steering_angle = VEHICLE.MAX_STEERING_ANGLE
        
    def calculate_steering(self, lane_center: float, lane_confidence: float) -> float:
        """
        Calculate steering angle for lane following
        lane_center: -1.0 (left) to +1.0 (right) of lane center
        lane_confidence: 0.0 to 1.0 confidence in lane detection
        """
        # Adjust error based on confidence
        effective_error = lane_center * min(1.0, lane_confidence * 1.5)
        
        # Calculate PID control
        steering_angle = self._calculate_pid(effective_error, self.lane_pid)
        
        # Apply steering limits
        steering_angle = max(-self.max_steering_angle, 
                           min(self.max_steering_angle, steering_angle))
        
        # Convert to percentage for vehicle control (-100% to +100%)
        steering_percent = (steering_angle / self.max_steering_angle) * 100.0
        
        return steering_percent
    
    def calculate_sign_avoidance(self, sign_color: str, sign_position: float) -> float:
        """
        Calculate steering to avoid traffic signs according to WRO rules
        sign_color: "red" or "green" 
        sign_position: -1.0 (left) to +1.0 (right) in camera view
        """
        # WRO rules: pass red signs on right, green signs on left
        if sign_color == "red":
            # Pass on right - steer left if sign is on right, steer right if sign is on left
            target_offset = -0.7  # Target position left of sign
        else:  # green
            # Pass on left - steer right if sign is on left, steer left if sign is on right  
            target_offset = 0.7   # Target position right of sign
        
        # Calculate error (difference from target position)
        error = target_offset - sign_position
        
        # Calculate PID control (more aggressive for sign avoidance)
        steering_angle = self._calculate_pid(error, self.sign_pid)
        
        # Apply limits and convert to percentage
        steering_angle = max(-self.max_steering_angle, 
                           min(self.max_steering_angle, steering_angle))
        steering_percent = (steering_angle / self.max_steering_angle) * 100.0
        
        return steering_percent
    
    def calculate_parking_steering(self, parking_center: float, parking_width: float) -> float:
        """
        Calculate steering for parallel parking maneuver
        parking_center: -1.0 to +1.0 position of parking spot center
        parking_width: relative width of parking spot (0.0 to 1.0)
        """
        # Simple parking algorithm:
        # 1. Align with parking spot
        # 2. Reverse with sharp turn
        # 3. Straighten when properly positioned
        
        # Phase 1: Alignment
        if abs(parking_center) > 0.1:  # Not yet aligned
            error = -parking_center  # Steer toward center
            steering_angle = self._calculate_pid(error, self.parking_pid)
        else:
            # Phase 2: Parking maneuver (simplified)
            steering_angle = 45.0 if parking_width > 0.6 else 30.0  # Sharp turn for wide spots
        
        # Convert to percentage
        steering_percent = (steering_angle / self.max_steering_angle) * 100.0
        return steering_percent
    
    def _calculate_pid(self, error: float, pid: PIDCoefficients) -> float:
        """Calculate PID control output"""
        import time
        current_time = time.time()
        
        if self.last_update_time == 0:
            self.last_update_time = current_time
            self.prev_error = error
            return error * pid.kp  # P-only on first call
        
        # Calculate time delta
        dt = current_time - self.last_update_time
        if dt <= 0:
            return 0.0
        
        self.last_update_time = current_time
        
        # Proportional term
        p_term = pid.kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = max(-pid.max_integral, min(pid.max_integral, self.integral))
        i_term = pid.ki * self.integral
        
        # Derivative term
        derivative = (error - self.prev_error) / dt
        d_term = pid.kd * derivative
        self.prev_error = error
        
        # PID output
        output = p_term + i_term + d_term
        
        return output
    
    def reset_controller(self) -> None:
        """Reset PID controller state"""
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_update_time = 0.0
    
    def get_steering_status(self) -> Dict[str, float]:
        """Get current steering controller status"""
        return {
            "prev_error": self.prev_error,
            "integral": self.integral,
            "max_steering_angle": self.max_steering_angle
        }

# Global steering controller instance
steering_controller = SteeringController()
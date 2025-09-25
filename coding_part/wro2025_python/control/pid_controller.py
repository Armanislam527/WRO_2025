"""
Generic PID controller implementation
Can be used for various control tasks beyond steering
"""

from dataclasses import dataclass
from typing import Callable, Optional

@dataclass
class PIDConfig:
    """Configuration for PID controller"""
    kp: float = 1.0
    ki: float = 0.0
    kd: float = 0.0
    max_output: float = 100.0
    min_output: float = -100.0
    max_integral: float = 100.0
    setpoint: float = 0.0

class PIDController:
    """
    Generic PID controller with anti-windup and output limiting
    Can be used for speed control, position control, etc.
    """
    
    def __init__(self, config: PIDConfig):
        self.config = config
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time: Optional[float] = None
        
    def update(self, process_variable: float, current_time: float) -> float:
        """
        Update PID controller and return control output
        process_variable: current measured value
        current_time: current time in seconds
        """
        error = self.config.setpoint - process_variable
        
        if self.last_time is None:
            self.last_time = current_time
            return error * self.config.kp  # P-only on first call
        
        # Calculate time delta
        dt = current_time - self.last_time
        if dt <= 0:
            return 0.0
        
        self.last_time = current_time
        
        # Proportional term
        p_term = self.config.kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = max(-self.config.max_integral, 
                          min(self.config.max_integral, self.integral))
        i_term = self.config.ki * self.integral
        
        # Derivative term
        derivative = (error - self.previous_error) / dt
        d_term = self.config.kd * derivative
        self.previous_error = error
        
        # Calculate output
        output = p_term + i_term + d_term
        
        # Apply output limits
        output = max(self.config.min_output, min(self.config.max_output, output))
        
        return output
    
    def reset(self) -> None:
        """Reset controller state"""
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = None
    
    def set_setpoint(self, setpoint: float) -> None:
        """Change the target setpoint"""
        self.config.setpoint = setpoint
    
    def update_coefficients(self, kp: float, ki: float, kd: float) -> None:
        """Update PID coefficients"""
        self.config.kp = kp
        self.config.ki = ki
        self.config.kd = kd

# Example usage for speed control
def create_speed_pid() -> PIDController:
    """Create a PID controller for speed regulation"""
    config = PIDConfig(
        kp=0.5,    # Proportional gain
        ki=0.01,   # Integral gain  
        kd=0.1,    # Derivative gain
        max_output=100.0,  # Max motor output %
        min_output=-100.0, # Min motor output %
        max_integral=50.0, # Anti-windup limit
        setpoint=0.0       # Target speed (will be set dynamically)
    )
    return PIDController(config)
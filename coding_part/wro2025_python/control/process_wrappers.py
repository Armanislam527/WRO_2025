"""
Process wrapper functions for control system
Handles navigation, motor control, and decision making
"""

import time
import sys
from typing import Any
from core.shared_memory import shared_memory, ControlState
from config.vehicle_config import RULES, VEHICLE, PERF

def control_process(stop_flag: Any, shutdown_flag: Any) -> None:
    """
    Control process - makes navigation decisions and sends commands to Nano
    Implements competition-specific logic for WRO 2025 challenges
    """
    print("🎮 Control process starting...")
    
    # Set process name
    try:
        import setproctitle
        setproctitle.setproctitle("wro2025_control")
    except ImportError:
        pass
    
    try:
        # Import control components
        from control.navigation_controller import NavigationController
        from control.motor_controller import MotorController
        from control.steering_controller import SteeringController
        from communication.serial_handler import serial_handler
        
        # Initialize control components
        nav_controller = NavigationController()
        motor_controller = MotorController()
        steering_controller = SteeringController()
        
        print("✅ Control components initialized")
        
        # Wait for required systems to be ready
        print("⏳ Waiting for Nano connection and camera...")
        while (not shared_memory.nano_connected.value or 
               not shared_memory.camera_ready.value) and not stop_flag.value:
            time.sleep(0.1)
        
        if stop_flag.value:
            return
            
        print("🚗 Control system ready - awaiting start signal")
        
        control_cycle = 0
        last_control_time = time.time()
        
        # Main control loop
        while not stop_flag.value and not shutdown_flag.value:
            cycle_start = time.time()
            control_cycle += 1
            
            # Get current system state
            sensor_data = shared_memory.get_sensor_data()
            vision_data = shared_memory.get_vision_data()
            current_state = shared_memory.get_control_state()
            
            # Skip processing if in emergency stop
            if current_state.emergency_stop:
                if control_cycle % 10 == 0:  # Log occasionally
                    print("🛑 Control system in emergency stop")
                time.sleep(0.1)
                continue
            
            # Make navigation decisions based on current mode
            new_state = ControlState()
            new_state.operation_mode = current_state.operation_mode
            
            if new_state.operation_mode == "IDLE":
                # Wait for start signal or manual override
                new_state.target_speed = 0
                new_state.target_steering = 0
                
                # Check if we should start (based on competition rules)
                if _should_start_mission(sensor_data, vision_data):
                    new_state.operation_mode = "LANE_FOLLOWING"
                    print("🏁 Starting mission - lane following mode")
                    
            elif new_state.operation_mode == "LANE_FOLLOWING":
                # Follow lanes while watching for traffic signs
                new_state.target_speed = motor_controller.calculate_speed(
                    vision_data.lane_confidence, 
                    sensor_data.front_distance
                )
                new_state.target_steering = steering_controller.calculate_steering(
                    vision_data.lane_center,
                    vision_data.lane_confidence
                )
                
                # Check for traffic signs
                if vision_data.traffic_sign_detected:
                    new_state.operation_mode = "SIGN_AVOIDANCE"
                    print("🚦 Traffic sign detected - avoidance mode")
                    
            elif new_state.operation_mode == "SIGN_AVOIDANCE":
                # Avoid traffic signs according to WRO rules
                new_state.target_speed = motor_controller.calculate_sign_approach_speed(
                    sensor_data.front_distance
                )
                new_state.target_steering = steering_controller.calculate_sign_avoidance(
                    vision_data.sign_color,
                    vision_data.sign_position
                )
                
                # Return to lane following after sign is passed
                if _sign_passed(sensor_data, vision_data):
                    new_state.operation_mode = "LANE_FOLLOWING"
                    print("✅ Sign passed - returning to lane following")
                    
            elif new_state.operation_mode == "PARKING":
                # Execute parallel parking maneuver
                new_state.target_speed = motor_controller.calculate_parking_speed(
                    sensor_data.front_distance,
                    sensor_data.back_distance
                )
                new_state.target_steering = steering_controller.calculate_parking_steering(
                    vision_data.parking_center,
                    vision_data.parking_width
                )
                
                if _parking_completed(sensor_data, vision_data):
                    new_state.operation_mode = "IDLE"
                    new_state.target_speed = 0
                    print("🅿️ Parking completed - mission finished")
            
            # Apply safety limits
            new_state.target_speed = max(-100, min(100, new_state.target_speed))
            new_state.target_steering = max(-100, min(100, new_state.target_steering))
            
            # Send commands to Nano
            if shared_memory.nano_connected.value:
                # Send motor command
                serial_handler.send_motor_command(new_state.target_speed)
                
                # Send steering command (convert from -100/+100 to 0-180 degrees)
                steering_angle = 90 + (new_state.target_steering * 0.9)  # 90° ± 90°
                serial_handler.send_servo_command(steering_angle)
            
            # Update shared memory with new control state
            shared_memory.update_control_state(new_state)
            
            # Log control decisions occasionally
            if control_cycle % 50 == 0:
                print(f"🎯 Control: Speed={new_state.target_speed:+.1f}%, "
                      f"Steering={new_state.target_steering:+.1f}%, "
                      f"Mode={new_state.operation_mode}")
            
            # Maintain control frequency (20Hz target)
            cycle_time = time.time() - cycle_start
            target_cycle_time = 1.0 / 20.0  # 50ms per cycle
            if cycle_time < target_cycle_time:
                time.sleep(target_cycle_time - cycle_time)
                
        print("🔴 Control process stopping...")
        
    except KeyboardInterrupt:
        print("🛑 Control process interrupted")
    except Exception as e:
        print(f"❌ Control process error: {e}")
        # Send emergency stop on control errors
        serial_handler.send_emergency_stop()
    finally:
        print("✅ Control process cleanup complete")

def _should_start_mission(sensor_data, vision_data) -> bool:
    """Determine if mission should start based on competition rules"""
    # Implementation depends on specific start conditions
    # For now, return False - will be triggered externally
    return False

def _sign_passed(sensor_data, vision_data) -> bool:
    """Determine if traffic sign has been successfully passed"""
    # Simple implementation based on distance and position
    return sensor_data.front_distance > 100  # Sign is 100cm away

def _parking_completed(sensor_data, vision_data) -> bool:
    """Determine if parking maneuver is completed"""
    # Simple implementation - will be enhanced with proper parking detection
    return sensor_data.front_distance < 20 and sensor_data.back_distance < 20
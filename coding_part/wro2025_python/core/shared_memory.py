"""
Thread-safe shared memory for inter-process communication
Optimized for Pi Zero 2W with minimal memory footprint
"""

import multiprocessing as mp
import threading
import time
from typing import Any, Dict, Optional
from dataclasses import dataclass, field
from config.vehicle_config import PERF, VEHICLE

@dataclass
class SensorData:
    """Unified sensor data structure matching Nano's CompactSensorData"""
    front_distance: int = 0
    right_distance: int = 0  
    back_distance: int = 0
    left_distance: int = 0
    accel_x: int = 0
    accel_y: int = 0
    accel_z: int = 0
    gyro_x: int = 0
    gyro_y: int = 0
    gyro_z: int = 0
    timestamp: float = 0.0

@dataclass
class VisionData:
    """Processed vision data for control decisions"""
    lane_center: float = 0.0  # -1.0 (left) to 1.0 (right)
    lane_confidence: float = 0.0  # 0.0 to 1.0
    traffic_sign_detected: bool = False
    sign_color: str = ""  # "red", "green", or ""
    sign_position: float = 0.0  # Relative position in frame
    parking_detected: bool = False
    parking_center: float = 0.0
    parking_width: float = 0.0
    frame_timestamp: float = 0.0

@dataclass
class ControlState:
    """Vehicle control commands and state"""
    target_speed: float = 0.0  # -100 to +100 %
    target_steering: float = 0.0  # -100 to +100 %
    emergency_stop: bool = False
    operation_mode: str = "IDLE"  # IDLE, LANE_FOLLOWING, SIGN_AVOIDANCE, PARKING
    last_heartbeat: float = 0.0

class SharedMemoryManager:
    """
    Manages thread-safe shared memory between processes
    Uses double-buffering to prevent read/write conflicts
    """
    
    def __init__(self):
        # Sensor data from Nano (updated by serial process)
        self.sensor_data = mp.Array('d', 11)  # 10 values + timestamp
        self.sensor_lock = mp.Lock()
        
        # Vision processing results (updated by vision process)  
        self.vision_data = mp.Array('d', 7)  # 6 values + timestamp
        self.vision_lock = mp.Lock()
        
        # Control commands (updated by control process)
        self.control_state = mp.Array('d', 5)  # 4 values + timestamp
        self.control_lock = mp.Lock()
        
        # System status flags
        self.system_ready = mp.Value('b', False)
        self.camera_ready = mp.Value('b', False)
        self.nano_connected = mp.Value('b', False)
        
        # Performance monitoring
        self.frame_counter = mp.Value('i', 0)
        self.processing_times = mp.Array('d', 10)  # Rolling window of processing times
        
    def update_sensor_data(self, data: SensorData) -> None:
        """Thread-safe update of sensor data from Nano"""
        with self.sensor_lock:
            self.sensor_data[0] = data.front_distance
            self.sensor_data[1] = data.right_distance
            self.sensor_data[2] = data.back_distance
            self.sensor_data[3] = data.left_distance
            self.sensor_data[4] = data.accel_x
            self.sensor_data[5] = data.accel_y
            self.sensor_data[6] = data.accel_z
            self.sensor_data[7] = data.gyro_x
            self.sensor_data[8] = data.gyro_y
            self.sensor_data[9] = data.gyro_z
            self.sensor_data[10] = data.timestamp
    
    def get_sensor_data(self) -> SensorData:
        """Thread-safe retrieval of sensor data"""
        with self.sensor_lock:
            data = SensorData()
            data.front_distance = int(self.sensor_data[0])
            data.right_distance = int(self.sensor_data[1])
            data.back_distance = int(self.sensor_data[2])
            data.left_distance = int(self.sensor_data[3])
            data.accel_x = int(self.sensor_data[4])
            data.accel_y = int(self.sensor_data[5])
            data.accel_z = int(self.sensor_data[6])
            data.gyro_x = int(self.sensor_data[7])
            data.gyro_y = int(self.sensor_data[8])
            data.gyro_z = int(self.sensor_data[9])
            data.timestamp = self.sensor_data[10]
            return data
    
    def update_vision_data(self, data: VisionData) -> None:
        """Thread-safe update of vision processing results"""
        with self.vision_lock:
            self.vision_data[0] = data.lane_center
            self.vision_data[1] = data.lane_confidence
            self.vision_data[2] = float(data.traffic_sign_detected)
            self.vision_data[3] = data.sign_position
            self.vision_data[4] = float(data.parking_detected)
            self.vision_data[5] = data.parking_center
            self.vision_data[6] = data.frame_timestamp
    
    def get_vision_data(self) -> VisionData:
        """Thread-safe retrieval of vision data"""
        with self.vision_lock:
            data = VisionData()
            data.lane_center = self.vision_data[0]
            data.lane_confidence = self.vision_data[1]
            data.traffic_sign_detected = bool(self.vision_data[2])
            data.sign_position = self.vision_data[3]
            data.parking_detected = bool(self.vision_data[4])
            data.parking_center = self.vision_data[5]
            data.frame_timestamp = self.vision_data[6]
            return data
    
    def update_control_state(self, state: ControlState) -> None:
        """Thread-safe update of control commands"""
        with self.control_lock:
            self.control_state[0] = state.target_speed
            self.control_state[1] = state.target_steering
            self.control_state[2] = float(state.emergency_stop)
            # Operation mode encoded as float (0:IDLE, 1:LANE_FOLLOWING, etc.)
            mode_map = {"IDLE": 0, "LANE_FOLLOWING": 1, "SIGN_AVOIDANCE": 2, "PARKING": 3}
            self.control_state[3] = mode_map.get(state.operation_mode, 0)
            self.control_state[4] = time.time()
    
    def get_control_state(self) -> ControlState:
        """Thread-safe retrieval of control state"""
        with self.control_lock:
            state = ControlState()
            state.target_speed = self.control_state[0]
            state.target_steering = self.control_state[1]
            state.emergency_stop = bool(self.control_state[2])
            mode_map = {0: "IDLE", 1: "LANE_FOLLOWING", 2: "SIGN_AVOIDANCE", 3: "PARKING"}
            state.operation_mode = mode_map.get(int(self.control_state[3]), "IDLE")
            state.last_heartbeat = self.control_state[4]
            return state
    
    def record_processing_time(self, process_time: float) -> None:
        """Record processing time for performance monitoring"""
        with self.control_lock:  # Reuse control lock for simplicity
            # Shift existing times and add new one
            for i in range(9, 0, -1):
                self.processing_times[i] = self.processing_times[i-1]
            self.processing_times[0] = process_time
    
    def get_average_processing_time(self) -> float:
        """Get average processing time over last 10 frames"""
        with self.control_lock:
            total = sum(self.processing_times)
            count = sum(1 for t in self.processing_times if t > 0)
            return total / count if count > 0 else 0.0
    # Add these methods to the SharedMemoryManager class:

    def get_vision_skip_frames(self) -> int:
        """Get current frame skipping setting for load management"""
        return max(0, int(self.processing_times[0]))  # Reuse array for settings
    
    def set_vision_skip_frames(self, skip_frames: int) -> None:
        """Set frame skipping for adaptive load management"""
        self.processing_times[0] = max(0, min(5, skip_frames))
    
    def get_target_fps(self) -> float:
        """Get target FPS for vision processing"""
        return max(5.0, min(30.0, self.processing_times[1]))  # Reuse array
    
    def set_target_fps(self, fps: float) -> None:
        """Set target FPS for vision processing"""
        self.processing_times[1] = max(5.0, min(30.0, fps))
# Global shared memory instance
shared_memory = SharedMemoryManager()
"""
Vision system integrator - coordinates all vision components
Provides unified interface for the control system
"""

import time
import numpy as np
from typing import Dict, Any
from core.shared_memory import VisionData
from vision.camera_pipeline import camera_pipeline
from vision.frame_processor import frame_processor
from vision.lane_detector import lane_detector
from vision.sign_detector import sign_detector
from vision.parking_detector import parking_detector

class VisionIntegrator:
    """
    Main vision system coordinator
    Manages all vision components and provides processed data to control system
    """
    
    def __init__(self):
        self.components_initialized = False
        self.last_processing_time = 0
        self.frame_count = 0
        self.average_processing_time = 0
        
        # Performance monitoring
        self.processing_times = []
        self.max_processing_times = 100
        
        # Operation modes
        self.parking_detection_enabled = False
        
        print("🔍 Vision integrator initializing...")
    
    def initialize(self) -> bool:
        """Initialize all vision components"""
        try:
            print("⚙️ Initializing vision components...")
            
            # Initialize camera pipeline
            if not camera_pipeline.initialize():
                print("❌ Camera pipeline initialization failed")
                return False
            
            # Start camera streaming
            if not camera_pipeline.start_streaming():
                print("❌ Camera streaming failed")
                return False
            
            # Wait for camera to be ready
            for _ in range(50):  # 5 second timeout
                if camera_pipeline.current_frame is not None:
                    break
                time.sleep(0.1)
            else:
                print("❌ Camera not ready after timeout")
                return False
            
            self.components_initialized = True
            print("✅ Vision system initialized successfully")
            return True
            
        except Exception as e:
            print(f"❌ Vision system initialization failed: {e}")
            return False
    
    def process_next_frame(self) -> VisionData:
        """
        Process the next available frame through the complete vision pipeline
        Returns VisionData structure for the control system
        """
        if not self.components_initialized:
            return VisionData()
        
        start_time = time.time()
        self.frame_count += 1
        
        try:
            # Capture current frame
            frame = camera_pipeline.capture_frame()
            if frame is None:
                return VisionData()
            
            # Initialize vision data structure
            vision_data = VisionData()
            vision_data.frame_timestamp = start_time
            
            # Process frame through the complete pipeline
            self._process_lane_detection(frame, vision_data)
            self._process_sign_detection(frame, vision_data)
            
            if self.parking_detection_enabled:
                self._process_parking_detection(frame, vision_data)
            
            # Calculate processing time
            processing_time = (time.time() - start_time) * 1000  # Convert to ms
            self.last_processing_time = processing_time
            
            # Update average processing time
            self.processing_times.append(processing_time)
            if len(self.processing_times) > self.max_processing_times:
                self.processing_times.pop(0)
            
            self.average_processing_time = sum(self.processing_times) / len(self.processing_times)
            
            return vision_data
            
        except Exception as e:
            print(f"❌ Vision processing error: {e}")
            return VisionData()
    
    def _process_lane_detection(self, frame: np.ndarray, vision_data: VisionData) -> None:
        """Process lane detection using specialized lane detector"""
        try:
            lane_center, confidence = lane_detector.detect_lanes(frame)
            vision_data.lane_center = lane_center
            vision_data.lane_confidence = confidence
            
        except Exception as e:
            print(f"❌ Lane detection error: {e}")
            vision_data.lane_center = 0.0
            vision_data.lane_confidence = 0.0
    
    def _process_sign_detection(self, frame: np.ndarray, vision_data: VisionData) -> None:
        """Process traffic sign detection using specialized sign detector"""
        try:
            signs = sign_detector.detect_signs(frame)
            
            if signs:
                # Use the most confident sign
                best_sign = max(signs, key=lambda x: x.confidence)
                if best_sign.confidence > 0.3:  # Confidence threshold
                    vision_data.traffic_sign_detected = True
                    vision_data.sign_color = best_sign.color
                    vision_data.sign_position = best_sign.position[0]  # x coordinate only
                    
        except Exception as e:
            print(f"❌ Sign detection error: {e}")
    
    def _process_parking_detection(self, frame: np.ndarray, vision_data: VisionData) -> None:
        """Process parking spot detection using specialized parking detector"""
        try:
            parking_spots = parking_detector.detect_parking_spots(frame)
            
            if parking_spots:
                # Use the most confident parking spot
                best_spot = max(parking_spots, key=lambda x: x.confidence)
                if best_spot.confidence > 0.4:  # Confidence threshold
                    vision_data.parking_detected = True
                    vision_data.parking_center = best_spot.center_position[0]  # x coordinate
                    vision_data.parking_width = best_spot.width
                    
        except Exception as e:
            print(f"❌ Parking detection error: {e}")
    
    def enable_parking_detection(self, enable: bool = True) -> None:
        """Enable or disable parking detection"""
        self.parking_detection_enabled = enable
        frame_processor.enable_parking_detection(enable)
        print(f"{'🅿️ Enabled' if enable else '❌ Disabled'} parking detection")
    
    def get_performance_stats(self) -> Dict[str, Any]:
        """Get vision system performance statistics"""
        camera_stats = camera_pipeline.get_frame_statistics()
        
        return {
            "camera_fps": camera_stats.get("current_fps", 0),
            "processing_fps": 1000 / self.average_processing_time if self.average_processing_time > 0 else 0,
            "average_processing_time_ms": self.average_processing_time,
            "frame_count": self.frame_count,
            "last_processing_time_ms": self.last_processing_time,
            "parking_detection_enabled": self.parking_detection_enabled,
            "components_initialized": self.components_initialized
        }
    
    def cleanup(self) -> None:
        """Cleanup vision system resources"""
        if self.components_initialized:
            camera_pipeline.cleanup()
            self.components_initialized = False
            print("✅ Vision system cleanup completed")

# Global vision integrator instance
vision_integrator = VisionIntegrator()
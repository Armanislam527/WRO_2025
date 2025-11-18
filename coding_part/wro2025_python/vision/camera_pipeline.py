"""
Camera pipeline for OV5647 module on Pi Zero 2W
Uses picamera2 for zero-copy video streaming with minimal CPU overhead
Optimized for WRO 2025 competition requirements
"""

import time
import numpy as np
from picamera2 import Picamera2
from picamera2.encoders import MJPEGEncoder
from picamera2.outputs import FileOutput
from config.camera_config import PIPELINE_CONFIG, PERF
from core.shared_memory import shared_memory

class OV5647Camera:
    """
    High-performance camera pipeline for OV5647 module
    Uses libcamera stack (picamera2) for best Raspberry Pi performance
    """
    
    def __init__(self):
        self.picam2 = None
        self.config = None
        self.frame_count = 0
        self.last_frame_time = 0
        self.fps = 0
        self.is_streaming = False
        
        # Frame buffers for zero-copy processing
        self.current_frame = None
        self.frame_timestamp = 0
        self.frame_lock = False  # Simple frame locking mechanism
        
    def initialize(self) -> bool:
        """Initialize the OV5647 camera module"""
        try:
            print("📷 Initializing OV5647 camera...")
            
            self.picam2 = Picamera2(camera_num=0)  # Explicitly use camera 0
            
            # Create optimized configuration for competition
            # Configure low-res RGB lores stream for direct array extraction
            self.config = self.picam2.create_video_configuration(
                main={
                    "size": (640, 480),
                    "format": "YUV420"  # Keep main lightweight
                },
                lores={
                    "size": (PERF.RESIZE_WIDTH, PERF.RESIZE_HEIGHT),
                    "format": "RGB888"  # Direct RGB frames for processing
                },
                display="lores",
                buffer_count=PERF.MAX_FRAME_BUFFERS,
                queue=False
            )
            
            # Set competition-optimized controls
            self.config["controls"] = {
                "FrameDurationLimits": (33333, 66666),  # 15-30 FPS
                "AwbMode": "auto",
                "AeEnable": True,
                "ExposureTime": 30000,  # 30ms exposure for indoor lighting
                "AnalogueGain": 4.0,
                "NoiseReductionMode": "fast",
                "Brightness": 0.1,  # Slightly brighter for better detection
                "Contrast": 1.2,    # Enhanced contrast for line detection
            }
            
            # Configure the camera
            self.picam2.configure(self.config)
            
            # Set up frame callback for zero-copy processing
            self.picam2.pre_callback = self._frame_callback
            
            print("✅ OV5647 camera initialized successfully")
            return True
            
        except Exception as e:
            print(f"❌ Camera initialization failed: {e}")
            return False
    
    def start_streaming(self) -> bool:
        """Start camera streaming"""
        try:
            if not self.picam2:
                return False
                
            print("🎥 Starting camera stream...")
            self.picam2.start()
            self.is_streaming = True
            self.last_frame_time = time.time()
            
            # Wait for first frame
            for _ in range(50):  # 50 attempts = 1 second timeout
                if self.current_frame is not None:
                    print("✅ Camera stream started with first frame received")
                    return True
                time.sleep(0.02)
            
            print("❌ Camera stream started but no frames received")
            return False
            
        except Exception as e:
            print(f"❌ Failed to start camera stream: {e}")
            return False
    
    def capture_frame(self) -> np.ndarray:
        """
        Capture the current frame for processing
        Returns None if no new frame available
        """
        if self.frame_lock or self.current_frame is None:
            return None
            
        self.frame_lock = True
        frame = self.current_frame.copy()  # Copy for thread safety
        self.frame_lock = False
        
        return frame
    
    def get_frame_statistics(self) -> dict:
        """Get camera performance statistics"""
        current_time = time.time()
        time_diff = current_time - self.last_frame_time if self.last_frame_time > 0 else 0
        
        return {
            "frame_count": self.frame_count,
            "current_fps": self.fps,
            "streaming": self.is_streaming,
            "last_frame_time": self.last_frame_time,
            "frame_width": PERF.RESIZE_WIDTH,
            "frame_height": PERF.RESIZE_HEIGHT
        }
    
    def _frame_callback(self, request):
        """Callback for new frames - zero-copy processing"""
        try:
            # Get the low-resolution RGB frame safely using Picamera2 API
            try:
                rgb_frame = request.make_array("lores")
            except Exception:
                # Fallback: attempt capture_array which copies frame
                rgb_frame = self.picam2.capture_array("lores")
                
            if rgb_frame is None or rgb_frame.size == 0:
                return
            
            # Update frame statistics
            current_time = time.time()
            self.frame_count += 1
            
            if self.last_frame_time > 0:
                self.fps = 1.0 / (current_time - self.last_frame_time)
            
            self.last_frame_time = current_time
            
            # Update shared frame (with simple locking)
            if not self.frame_lock:
                # Ensure a copy to avoid holding onto internal buffers
                self.current_frame = rgb_frame.copy()
                self.frame_timestamp = current_time
                
        except Exception as e:
            print(f"❌ Frame callback error: {e}")
    
    # Removed manual YUV conversion; using Picamera2-provided RGB lores frames
    
    def cleanup(self):
        """Cleanup camera resources"""
        if self.picam2 and self.is_streaming:
            self.picam2.stop()
            self.is_streaming = False
            print("✅ Camera cleanup completed")

# Global camera instance
camera_pipeline = OV5647Camera()
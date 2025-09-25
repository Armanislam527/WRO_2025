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
            self.config = self.picam2.create_video_configuration(
                main={
                    "size": (1296, 972),  # Use full 5MP for best quality
                    "format": "RGB888"
                },
                lores={
                    "size": (PERF.RESIZE_WIDTH, PERF.RESIZE_HEIGHT),  # Processing resolution
                    "format": "YUV420"  # More efficient for processing
                },
                display="lores",  # Display lower resolution to save resources
                buffer_count=PERF.MAX_FRAME_BUFFERS,  # Minimal buffers
                queue=False  # No frame queue to reduce latency
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
            # Get the low-resolution frame for processing
            buffer = request.request.buffers[1]  # lores buffer
            frame_data = buffer.data
            
            # Convert YUV420 to RGB for processing
            # This is optimized for Pi Zero 2W - uses numpy for efficiency
            yuv_array = np.frombuffer(frame_data, dtype=np.uint8)
            yuv_array = yuv_array.reshape((PERF.RESIZE_HEIGHT * 3 // 2, PERF.RESIZE_WIDTH))
            
            # Convert YUV to RGB (simplified conversion for performance)
            rgb_frame = self._yuv420_to_rgb(yuv_array, PERF.RESIZE_WIDTH, PERF.RESIZE_HEIGHT)
            
            # Update frame statistics
            current_time = time.time()
            self.frame_count += 1
            
            if self.last_frame_time > 0:
                self.fps = 1.0 / (current_time - self.last_frame_time)
            
            self.last_frame_time = current_time
            
            # Update shared frame (with simple locking)
            if not self.frame_lock:
                self.current_frame = rgb_frame
                self.frame_timestamp = current_time
                
        except Exception as e:
            print(f"❌ Frame callback error: {e}")
    
    def _yuv420_to_rgb(self, yuv_array, width, height):
        """Optimized YUV420 to RGB conversion for Pi Zero 2W"""
        # This is a simplified conversion for performance
        # For competition, we can use a more accurate conversion if needed
        
        # Extract Y, U, V components
        y = yuv_array[:height, :width]
        u = yuv_array[height:height + height // 2, :width // 2]
        v = yuv_array[height + height // 2:, :width // 2]
        
        # Upsample U and V to full resolution
        u_upsampled = np.repeat(np.repeat(u, 2, axis=0), 2, axis=1)
        v_upsampled = np.repeat(np.repeat(v, 2, axis=0), 2, axis=1)
        
        # Convert to RGB (simplified matrix multiplication)
        r = y + 1.402 * (v_upsampled - 128)
        g = y - 0.344136 * (u_upsampled - 128) - 0.714136 * (v_upsampled - 128)
        b = y + 1.772 * (u_upsampled - 128)
        
        # Stack channels and clamp values
        rgb = np.stack([r, g, b], axis=2)
        rgb = np.clip(rgb, 0, 255).astype(np.uint8)
        
        return rgb
    
    def cleanup(self):
        """Cleanup camera resources"""
        if self.picam2 and self.is_streaming:
            self.picam2.stop()
            self.is_streaming = False
            print("✅ Camera cleanup completed")

# Global camera instance
camera_pipeline = OV5647Camera()
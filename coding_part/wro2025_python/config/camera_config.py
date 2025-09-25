"""
OV5647 camera configuration optimized for Pi Zero 2W
Uses libcamera (picamera2) for best performance on limited hardware
"""

from picamera2 import Picamera2
from dataclasses import dataclass
from vehicle_config import PERF

@dataclass
class OV5647Config:
    """OV5647-specific camera configuration"""
    SENSOR_MODES = {
        'high_speed': {
            'size': (640, 480),  # VGA mode for processing
            'fps': 30,
            'bit_depth': 8
        },
        'high_quality': {
            'size': (1296, 972),  # Native 5MP but slower
            'fps': 15,
            'bit_depth': 10
        }
    }
    
    # OV5647 specific tuning
    ANALOG_GAIN_RANGE = (1.0, 16.0)
    DIGITAL_GAIN_RANGE = (1.0, 4.0)
    EXPOSURE_TIME_RANGE = (100, 100000)  # microseconds

class CameraPipelineConfig:
    """Camera configuration optimized for lane and sign detection"""
    
    @staticmethod
    def create_optimized_config():
        """Create camera config optimized for Pi Zero 2W"""
        config = Picamera2.create_video_configuration(
            main={
                'size': (PERF.RESIZE_WIDTH, PERF.RESIZE_HEIGHT),  # Low res for processing
                'format': 'RGB888'
            },
            lores={'size': (320, 240), 'format': 'YUV420'},  # Even lower res for preview
            display='lores',  # Display the lower resolution stream
            buffer_count=PERF.MAX_FRAME_BUFFERS,  # Minimal buffers to save memory
            queue=False  # Don't queue frames to reduce latency
        )
        
        # Tuning for better performance on OV5647
        config['controls'] = {
            'FrameRate': PERF.MAX_FPS,
            'AwbMode': 'auto',  # Auto white balance
            'AeEnable': True,   # Auto exposure
            'ExposureTime': 20000,  # Start with 20ms exposure
            'AnalogueGain': 4.0,
            'NoiseReductionMode': 'fast',  # Balance between quality and speed
        }
        
        return config

class VisionProcessingConfig:
    """Vision processing parameters optimized for competition environment"""
    
    # Color ranges for WRO 2025 competition (CMYK values from rules)
    COLOR_RANGES = {
        'orange_line': {
            'lower': [10, 100, 100],   # Approximate CMYK (0, 60, 100, 0) in RGB
            'upper': [50, 255, 255]    # HSV ranges
        },
        'blue_line': {
            'lower': [100, 150, 50],   # Approximate CMYK (100, 80, 0, 0)
            'upper': [130, 255, 200]
        },
        'red_sign': {
            'lower': [0, 100, 100],    # RGB (238, 39, 55)
            'upper': [10, 255, 255]
        },
        'green_sign': {
            'lower': [40, 50, 50],     # RGB (68, 214, 44)  
            'upper': [80, 255, 255]
        },
        'magenta_parking': {
            'lower': [140, 100, 100],  # RGB (255, 0, 255)
            'upper': [160, 255, 255]
        }
    }
    
    # Processing parameters
    GAUSSIAN_BLUR = (5, 5)  # Kernel size for noise reduction
    CANNY_THRESHOLD1 = 50
    CANNY_THRESHOLD2 = 150
    HOUGH_RHO = 1
    HOUGH_THETA = 3.141592 / 180  # 1 degree in radians
    HOUGH_THRESHOLD = 20

# Global configuration instances
CAMERA_CONFIG = OV5647Config()
PIPELINE_CONFIG = CameraPipelineConfig()
VISION_CONFIG = VisionProcessingConfig()
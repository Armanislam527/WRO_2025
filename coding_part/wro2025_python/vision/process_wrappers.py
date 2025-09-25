"""
Process wrapper functions for vision system - FINAL VERSION
Now uses the complete vision integrator for robust processing
"""

import time
import sys
from typing import Any
from core.shared_memory import shared_memory
from vision.vision_integrator import vision_integrator
from communication.heartbeat_monitor import heartbeat_monitor

def camera_process(stop_flag: Any, shutdown_flag: Any) -> None:
    """
    Camera process - handles camera initialization and frame provisioning
    Simplified version since camera is now managed by vision_integrator
    """
    print("📷 Camera process starting...")
    
    # Set process name for better debugging
    try:
        import setproctitle
        setproctitle.setproctitle("wro2025_camera")
    except ImportError:
        pass
    
    try:
        # Initialize vision system (which includes camera)
        if not vision_integrator.initialize():
            print("❌ Vision system initialization failed")
            sys.exit(1)
        
        # Mark camera as ready
        shared_memory.camera_ready.value = True
        print("✅ Camera process initialization complete")
        
        # Simple loop - camera is managed by vision_integrator
        while not stop_flag.value and not shutdown_flag.value:
            # Just keep the process alive, camera runs independently
            time.sleep(0.1)
            
        print("🔴 Camera process stopping...")
        
    except KeyboardInterrupt:
        print("🛑 Camera process interrupted")
    except Exception as e:
        print(f"❌ Camera process error: {e}")
    finally:
        vision_integrator.cleanup()
        print("✅ Camera process cleanup complete")

def vision_process(stop_flag: Any, shutdown_flag: Any) -> None:
    """
    Vision processing process - FINAL VERSION
    Uses the complete vision integrator for robust processing
    """
    print("👁️ Vision process starting...")
    
    # Set process name
    try:
        import setproctitle
        setproctitle.setproctitle("wro2025_vision")
    except ImportError:
        pass
    
    try:
        # Wait for camera to be ready
        print("⏳ Waiting for camera initialization...")
        while not shared_memory.camera_ready.value and not stop_flag.value:
            time.sleep(0.1)
        
        if stop_flag.value:
            return
            
        print("🔍 Vision process starting main processing loop...")
        
        frame_count = 0
        last_stat_time = time.time()
        skip_frames = 0
        
        # Main vision processing loop
        while not stop_flag.value and not shutdown_flag.value:
            cycle_start = time.time()
            
            # Adaptive frame skipping for CPU load management
            skip_frames += 1
            current_skip = shared_memory.get_vision_skip_frames()
            if skip_frames <= current_skip:
                time.sleep(0.005)
                continue
            skip_frames = 0
            
            # Process frame through complete vision pipeline
            vision_data = vision_integrator.process_next_frame()
            
            # Update shared memory with vision results
            shared_memory.update_vision_data(vision_data)
            
            # Update heartbeat for vision system
            heartbeat_monitor.update_vision_heartbeat()
            
            frame_count += 1
            shared_memory.frame_counter.value += 1
            
            # Calculate and record processing time
            processing_time = (time.time() - cycle_start) * 1000
            shared_memory.record_processing_time(processing_time)
            
            # Adaptive frame skipping based on processing load
            if processing_time > 66.0 and current_skip < 5:  # If slower than 15 FPS
                new_skip = current_skip + 1
                shared_memory.set_vision_skip_frames(new_skip)
                print(f"⚠️ High processing time: {processing_time:.1f}ms, skipping {new_skip} frames")
            elif processing_time < 33.0 and current_skip > 0:
                new_skip = current_skip - 1
                shared_memory.set_vision_skip_frames(new_skip)
            
            # Log performance every 5 seconds
            current_time = time.time()
            if current_time - last_stat_time > 5.0:
                stats = vision_integrator.get_performance_stats()
                avg_time = shared_memory.get_average_processing_time()
                print(f"👁️ Vision: {stats['processing_fps']:.1f} FPS, "
                      f"Avg: {avg_time:.1f}ms, Frames: {frame_count}")
                last_stat_time = current_time
            
            # Sleep to maintain target frame rate
            elapsed = time.time() - cycle_start
            target_time = 1.0 / shared_memory.get_target_fps()
            if elapsed < target_time:
                time.sleep(target_time - elapsed)
                
        print("🔴 Vision process stopping...")
        
    except KeyboardInterrupt:
        print("🛑 Vision process interrupted")
    except Exception as e:
        print(f"❌ Vision process error: {e}")
    finally:
        print("✅ Vision process cleanup complete")
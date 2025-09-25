#!/usr/bin/env python3
"""
WRO 2025 Future Engineers - Main Application Entry Point
FINAL INTEGRATED VERSION
"""

import os
import sys
import time
import signal
import multiprocessing as mp
from datetime import datetime

# Add the project root to Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from core.process_manager import process_manager
from core.state_machine import state_machine
from utils.performance_monitor import performance_optimizer
from utils.logger import competition_logger
from config.vehicle_config import PERF

def signal_handler(signum, frame):
    """Handle shutdown signals gracefully"""
    print(f"\n🛑 Received signal {signum}, initiating shutdown...")
    process_manager.graceful_shutdown()
    sys.exit(0)

def setup_environment() -> bool:
    """Setup system environment for optimal performance"""
    competition_logger.log_system_start()
    
    try:
        # Set up signal handlers
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        # Initialize performance optimizer
        performance_optimizer.optimize_pi_zero()
        
        # Set multiprocessing start method
        mp.set_start_method('spawn', force=True)
        
        print("✅ Environment setup complete")
        return True
        
    except Exception as e:
        print(f"❌ Environment setup failed: {e}")
        return False

def main() -> None:
    """Main application entry point"""
    print("=" * 60)
    print("🤖 WRO 2025 Future Engineers - Autonomous Vehicle System")
    print(f"📋 Pi Zero 2W - Target FPS: {PERF.MAX_FPS}")
    print(f"📋 Processing Resolution: {PERF.RESIZE_WIDTH}x{PERF.RESIZE_HEIGHT}")
    print(f"📋 Started at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 60)
    
    # Setup environment
    if not setup_environment():
        sys.exit(1)
    
    # Check if we're running on Raspberry Pi
    if not os.path.exists('/proc/device-tree/model'):
        print("⚠️  Not running on Raspberry Pi - some features may be limited")
    
    # Check for camera
    if not os.path.exists('/dev/video0'):
        print("⚠️  Camera not detected - vision processing will be limited")
    
    # Import process functions
    from communication.process_wrappers import serial_process
    from vision.process_wrappers import camera_process, vision_process
    from control.process_wrappers import control_process
    
    # Create processes
    print("👥 Creating parallel processes...")
    try:
        process_manager.create_process('serial', serial_process)
        process_manager.create_process('camera', camera_process) 
        process_manager.create_process('vision', vision_process)
        process_manager.create_process('control', control_process)
        
        print(f"✅ Created {len(process_manager.processes)} processes")
        
    except Exception as e:
        print(f"❌ Process creation failed: {e}")
        sys.exit(1)
    
    # Start all processes
    print("🚀 Starting all processes...")
    if not process_manager.start_all():
        print("❌ Failed to start processes")
        process_manager.emergency_shutdown()
        sys.exit(1)
    
    # Main system loop
    print("📊 Starting main system loop...")
    last_performance_log = time.time()
    last_state_log = time.time()
    
    try:
        while True:
            current_time = time.time()
            
            # Update state machine
            current_state = state_machine.update()
            
            # Log state every 10 seconds
            if current_time - last_state_log > 10.0:
                state_info = state_machine.get_state_info()
                print(f"🎯 Current state: {state_info['current_state']}, "
                      f"Lap: {state_info['lap_count']}, "
                      f"Score: {state_info['score_estimate']}")
                last_state_log = current_time
            
            # Log performance every 30 seconds
            if current_time - last_performance_log > 30.0:
                performance_data = performance_optimizer.monitor_performance()
                competition_logger.log_vehicle_performance(performance_data)
                
                print(f"📈 Performance: CPU {performance_data.get('cpu_percent', 0):.1f}%, "
                      f"Memory {performance_data.get('memory_percent', 0):.1f}%")
                last_performance_log = current_time
            
            # Check system health
            if not process_manager.health_check():
                print("❌ System health check failed")
                state_machine._transition_to(state_machine.EMERGENCY_STOP, "System health failure")
            
            # Small delay to prevent busy waiting
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n🛑 Keyboard interrupt received")
    except Exception as e:
        print(f"❌ Unexpected error in main loop: {e}")
        competition_logger.log_emergency("SYSTEM_FAILURE", str(e))
    finally:
        print("🔴 Shutting down WRO 2025 system...")
        competition_logger.save_performance_report()
        process_manager.graceful_shutdown()
        print("✅ WRO 2025 system shutdown complete")

if __name__ == "__main__":
    main()
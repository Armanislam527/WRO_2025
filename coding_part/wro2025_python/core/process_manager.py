"""
Parallel process management optimized for Pi Zero 2W
Handles process lifecycle, health monitoring, and graceful shutdown
"""

import multiprocessing as mp
import time
import signal
import sys
from typing import Dict, List
from config.vehicle_config import PERF
from core.shared_memory import shared_memory

class ProcessManager:
    """
    Manages parallel processes with health monitoring and graceful shutdown
    Optimized for limited CPU resources on Pi Zero 2W
    """
    
    def __init__(self):
        self.processes: Dict[str, mp.Process] = {}
        self.process_flags: Dict[str, mp.Value] = {}
        self.startup_order = ['serial', 'camera', 'vision', 'control']
        self.shutdown_flag = mp.Value('b', False)
        
    # Update the create_process method to use our new wrapper functions:

    def create_process(self, name: str, target_function, args: tuple = ()) -> None:
        """Create a process with proper error handling"""
        if name in self.processes:
            raise ValueError(f"Process {name} already exists")
            
        # Create stop flag for this process
        stop_flag = mp.Value('b', False)
        self.process_flags[name] = stop_flag
        
        # Map process names to their wrapper functions
        process_functions = {
            'serial': self._get_serial_process,
            'camera': self._get_camera_process, 
            'vision': self._get_vision_process,
            'control': self._get_control_process
        }
        
        if name in process_functions:
            target_function = process_functions[name]()
        
        # Create the process
        process = mp.Process(
            name=name,
            target=target_function,
            args=(stop_flag, self.shutdown_flag),
            daemon=False
        )
        
        self.processes[name] = process
    
    def _get_serial_process(self):
        """Get serial process function with proper imports"""
        from communication.process_wrappers import serial_process
        return serial_process
    
    def _get_camera_process(self):
        """Get camera process function"""
        from vision.process_wrappers import camera_process
        return camera_process
    
    def _get_vision_process(self):
        """Get vision process function"""
        from vision.process_wrappers import vision_process
        return vision_process
    
    def _get_control_process(self):
        """Get control process function"""
        from control.process_wrappers import control_process
        return control_process
    def start_all(self) -> bool:
        """Start all processes in proper order with health checks"""
        print("🚀 Starting WRO 2025 Vehicle System...")
        
        # Set up signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        try:
            # Start processes in defined order with delays
            for name in self.startup_order:
                if name in self.processes:
                    print(f"Starting {name} process...")
                    self.processes[name].start()
                    
                    # Wait for process to initialize
                    time.sleep(0.5)
                    
                    # Check if process started successfully
                    if not self.processes[name].is_alive():
                        print(f"❌ Process {name} failed to start")
                        return False
            
            # Wait for system to become ready
            return self.wait_for_system_ready()
            
        except Exception as e:
            print(f"❌ Error starting processes: {e}")
            self.emergency_shutdown()
            return False
    
    def wait_for_system_ready(self, timeout: int = 10) -> bool:
        """Wait for all subsystems to report ready"""
        print("⏳ Waiting for system readiness...")
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            # Check if all critical systems are ready
            if (shared_memory.nano_connected.value and 
                shared_memory.camera_ready.value and
                shared_memory.system_ready.value):
                print("✅ All systems ready!")
                return True
                
            time.sleep(0.1)
        
        print("❌ System readiness timeout")
        return False
    
    def monitor_processes(self) -> None:
        """Main monitoring loop - checks process health and performance"""
        print("📊 Starting process monitor...")
        
        last_health_check = time.time()
        frame_count = 0
        
        while not self.shutdown_flag.value:
            current_time = time.time()
            
            # Health check every 2 seconds
            if current_time - last_health_check > 2.0:
                if not self.health_check():
                    print("❌ Health check failed, initiating shutdown...")
                    self.graceful_shutdown()
                    break
                last_health_check = current_time
            
            # Performance monitoring
            frame_count = shared_memory.frame_counter.value
            avg_process_time = shared_memory.get_average_processing_time()
            
            # Log performance every 10 seconds
            if current_time % 10 < 0.1:  # Roughly every 10 seconds
                self.log_performance(frame_count, avg_process_time)
            
            time.sleep(0.5)  # Reduce monitoring frequency to save CPU
    
    def health_check(self) -> bool:
        """Check health of all processes"""
        all_healthy = True
        
        for name, process in self.processes.items():
            if not process.is_alive():
                print(f"❌ Process {name} is not running")
                all_healthy = False
            else:
                # Process-specific health checks
                if name == 'serial' and not shared_memory.nano_connected.value:
                    print(f"⚠️  Process {name} running but Nano not connected")
                elif name == 'camera' and not shared_memory.camera_ready.value:
                    print(f"⚠️  Process {name} running but camera not ready")
        
        return all_healthy
    
    def log_performance(self, frame_count: int, avg_process_time: float) -> None:
        """Log system performance statistics"""
        fps = frame_count / 10.0  # Frames per second over last 10 seconds
        cpu_warning = "⚠️" if avg_process_time > 66 else "✅"  # 15 FPS target
        
        print(f"📈 Performance: {fps:.1f} FPS, "
              f"Processing: {avg_process_time:.1f}ms {cpu_warning}")
        
        # Reset frame counter for next interval
        shared_memory.frame_counter.value = 0
    
    def signal_handler(self, signum, frame) -> None:
        """Handle shutdown signals gracefully"""
        print(f"🛑 Received signal {signum}, initiating shutdown...")
        self.graceful_shutdown()
    
    def graceful_shutdown(self) -> None:
        """Gracefully shutdown all processes"""
        print("🔴 Beginning graceful shutdown...")
        self.shutdown_flag.value = True
        
        # Signal all processes to stop
        for flag in self.process_flags.values():
            flag.value = True
        
        # Wait for processes to terminate
        timeout = 5.0  # 5 seconds
        start_time = time.time()
        
        for name, process in self.processes.items():
            if process.is_alive():
                print(f"Waiting for {name} to terminate...")
                process.join(timeout - (time.time() - start_time))
                
                if process.is_alive():
                    print(f"⚠️  Process {name} didn't terminate gracefully, forcing...")
                    process.terminate()
        
        print("✅ All processes stopped")
        sys.exit(0)
    
    def emergency_shutdown(self) -> None:
        """Emergency shutdown without graceful waiting"""
        print("🚨 EMERGENCY SHUTDOWN INITIATED!")
        self.shutdown_flag.value = True
        
        for name, process in self.processes.items():
            if process.is_alive():
                process.terminate()
        
        sys.exit(1)

# Global process manager instance
process_manager = ProcessManager()
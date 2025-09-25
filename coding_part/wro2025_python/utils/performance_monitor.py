"""
Performance monitoring and system optimization for Pi Zero 2W
"""

import os
import time
import psutil
from typing import Dict, Any

class PerformanceOptimizer:
    """
    System performance optimizer for Pi Zero 2W
    Applies optimizations for real-time vehicle control
    """
    
    def __init__(self):
        self.initial_optimizations_applied = False
        self.start_time = time.time()
    
    def optimize_pi_zero(self) -> bool:
        """Apply Pi Zero 2W specific optimizations"""
        try:
            if self.initial_optimizations_applied:
                return True
                
            print("⚙️ Applying Pi Zero 2W optimizations...")
            
            # Set CPU governor to performance
            self._set_cpu_governor("performance")
            
            # Set nice priority for current process
            os.nice(-10)
            
            # Reduce swapiness to prefer RAM
            self._set_swappiness(10)
            
            print("✅ Pi Zero 2W optimizations applied")
            self.initial_optimizations_applied = True
            return True
            
        except Exception as e:
            print(f"❌ Performance optimization failed: {e}")
            return False
    
    def _set_cpu_governor(self, governor: str) -> None:
        """Set CPU governor for all cores"""
        try:
            for cpu in range(psutil.cpu_count()):
                governor_path = f"/sys/devices/system/cpu/cpu{cpu}/cpufreq/scaling_governor"
                if os.path.exists(governor_path):
                    with open(governor_path, 'w') as f:
                        f.write(governor)
        except Exception as e:
            print(f"⚠️ Could not set CPU governor: {e}")
    
    def _set_swappiness(self, value: int) -> None:
        """Set system swappiness"""
        try:
            with open("/proc/sys/vm/swappiness", 'w') as f:
                f.write(str(value))
        except Exception as e:
            print(f"⚠️ Could not set swappiness: {e}")
    
    def monitor_performance(self) -> Dict[str, Any]:
        """Monitor current system performance"""
        try:
            # CPU usage
            cpu_percent = psutil.cpu_percent(interval=0.1)
            
            # Memory usage
            memory = psutil.virtual_memory()
            
            # Disk I/O
            disk_io = psutil.disk_io_counters()
            
            return {
                "timestamp": time.time(),
                "cpu_percent": cpu_percent,
                "memory_percent": memory.percent,
                "memory_used_mb": memory.used // (1024 * 1024),
                "disk_read_mb": disk_io.read_bytes // (1024 * 1024) if disk_io else 0,
                "disk_write_mb": disk_io.write_bytes // (1024 * 1024) if disk_io else 0,
                "uptime": time.time() - self.start_time
            }
            
        except Exception as e:
            print(f"❌ Performance monitoring error: {e}")
            return {}
    
    def check_for_throttling(self) -> bool:
        """Check if the CPU is being throttled"""
        try:
            with open("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq", "r") as f:
                current_freq = int(f.read().strip())
            
            with open("/sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq", "r") as f:
                max_freq = int(f.read().strip())
            
            # If current frequency is significantly lower than max, we might be throttling
            return current_freq < max_freq * 0.8
            
        except Exception as e:
            print(f"⚠️ Could not check for throttling: {e}")
            return False

# Global performance optimizer instance
performance_optimizer = PerformanceOptimizer()
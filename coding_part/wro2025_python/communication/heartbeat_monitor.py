"""
Heartbeat monitoring and system health checks
Ensures continuous communication with Nano and triggers safety measures on failure
"""

import time
import threading
from typing import Callable, Optional
from core.shared_memory import shared_memory
from config.serial_config import SAFETY_CONF

class HeartbeatMonitor:
    """
    Monitors system health and triggers emergency procedures on failures
    Works in conjunction with the serial handler's heartbeat system
    """
    
    def __init__(self):
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.last_nano_heartbeat = 0.0
        self.last_vision_heartbeat = 0.0
        self.emergency_callbacks = []
        
    def start(self) -> bool:
        """Start the heartbeat monitoring thread"""
        self.running = True
        self.thread = threading.Thread(target=self._monitoring_loop, daemon=True)
        self.thread.start()
        print("❤️  Heartbeat monitor started")
        return True
    
    def stop(self) -> None:
        """Stop heartbeat monitoring"""
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        print("🛑 Heartbeat monitor stopped")
    
    def register_emergency_callback(self, callback: Callable) -> None:
        """Register callback for emergency situations"""
        self.emergency_callbacks.append(callback)
    
    def update_nano_heartbeat(self) -> None:
        """Update Nano heartbeat timestamp (called by serial handler)"""
        self.last_nano_heartbeat = time.time()
    
    def update_vision_heartbeat(self) -> None:
        """Update vision heartbeat timestamp"""
        self.last_vision_heartbeat = time.time()
    
    def _check_nano_connection(self) -> bool:
        """Check if Nano is responding within expected timeframe"""
        current_time = time.time()
        time_since_heartbeat = current_time - self.last_nano_heartbeat
        
        if time_since_heartbeat > SAFETY_CONF.COMMUNICATION_TIMEOUT_MS / 1000.0:
            print(f"❌ Nano heartbeat lost for {time_since_heartbeat:.1f}s")
            return False
        elif time_since_heartbeat > SAFETY_CONF.COMMUNICATION_TIMEOUT_MS / 2000.0:  # Half timeout
            print(f"⚠️  Nano heartbeat delay: {time_since_heartbeat:.1f}s")
            
        return True
    
    def _check_vision_health(self) -> bool:
        """Check if vision system is processing frames regularly"""
        current_time = time.time()
        time_since_vision = current_time - self.last_vision_heartbeat
        
        # Vision can have longer timeout since processing is heavier
        vision_timeout = SAFETY_CONF.COMMUNICATION_TIMEOUT_MS * 2 / 1000.0
        
        if time_since_vision > vision_timeout:
            print(f"❌ Vision system stalled for {time_since_vision:.1f}s")
            return False
            
        return True
    
    def _trigger_emergency(self, reason: str) -> None:
        """Trigger emergency procedures"""
        print(f"🚨 EMERGENCY TRIGGERED: {reason}")
        
        # Call all registered emergency callbacks
        for callback in self.emergency_callbacks:
            try:
                callback(reason)
            except Exception as e:
                print(f"❌ Emergency callback error: {e}")
    
    def _monitoring_loop(self) -> None:
        """Main monitoring loop"""
        print("🔍 Starting heartbeat monitoring...")
        
        emergency_triggered = False
        
        while self.running:
            try:
                current_time = time.time()
                
                # Check Nano connection
                nano_healthy = self._check_nano_connection()
                
                # Check vision system (if it should be running)
                vision_healthy = True
                if shared_memory.system_ready.value and shared_memory.camera_ready.value:
                    vision_healthy = self._check_vision_health()
                
                # Trigger emergency if critical systems fail
                if not nano_healthy and not emergency_triggered:
                    self._trigger_emergency("Nano communication lost")
                    emergency_triggered = True
                elif not vision_healthy and not emergency_triggered:
                    self._trigger_emergency("Vision system stalled")
                    emergency_triggered = True
                elif nano_healthy and vision_healthy and emergency_triggered:
                    print("✅ System recovered from emergency state")
                    emergency_triggered = False
                
                # Update shared memory status
                shared_memory.nano_connected.value = nano_healthy
                
                # Sleep to reduce CPU load (check every 500ms)
                time.sleep(0.5)
                
            except Exception as e:
                print(f"❌ Heartbeat monitor error: {e}")
                time.sleep(1.0)
        
        print("🔴 Heartbeat monitoring stopped")

# Global heartbeat monitor instance
heartbeat_monitor = HeartbeatMonitor()
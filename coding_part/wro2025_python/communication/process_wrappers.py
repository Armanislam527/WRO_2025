"""
Process wrapper functions for serial communication and heartbeat monitoring
Integrates with the core process manager for parallel execution
"""

import time
import signal
import sys
from typing import Any
from core.shared_memory import shared_memory
from communication.serial_handler import serial_handler, SerialHandler
from communication.heartbeat_monitor import heartbeat_monitor, HeartbeatMonitor

def serial_process(stop_flag: Any, shutdown_flag: Any) -> None:
    """
    Main process function for serial communication with Nano
    Runs in a separate process for robust communication handling
    """
    print("🔌 Serial process starting...")
    
    # Set process name for better debugging
    try:
        import setproctitle
        setproctitle.setproctitle("wro2025_serial")
    except ImportError:
        pass  # Optional dependency
    
    def emergency_callback(reason: str) -> None:
        """Handle emergency situations from heartbeat monitor"""
        print(f"🚨 Serial process emergency: {reason}")
        # Update control state to emergency stop
        from core.shared_memory import ControlState
        emergency_state = ControlState()
        emergency_state.emergency_stop = True
        emergency_state.target_speed = 0
        emergency_state.target_steering = 90  # Center steering
        shared_memory.update_control_state(emergency_state)
        
        # Send emergency stop to Nano
        serial_handler.send_emergency_stop()
    
    def nano_connected_callback(connected: bool) -> None:
        """Handle Nano connection status changes"""
        if connected:
            print("✅ Nano connection established in process")
            shared_memory.nano_connected.value = True
        else:
            print("❌ Nano connection lost in process")
            shared_memory.nano_connected.value = False
    
    try:
        # Register emergency callback
        heartbeat_monitor.register_emergency_callback(emergency_callback)
        
        # Register Nano connection callback
        serial_handler.register_callback('nano_connected', nano_connected_callback)
        
        # Start heartbeat monitor first (it will wait for serial)
        if not heartbeat_monitor.start():
            print("❌ Failed to start heartbeat monitor")
            sys.exit(1)
        
        # Start serial communication
        if not serial_handler.start():
            print("❌ Failed to start serial handler")
            sys.exit(1)
        
        # Mark serial system as ready
        shared_memory.system_ready.value = True
        print("✅ Serial process initialization complete")
        
        # Main process loop
        while not stop_flag.value and not shutdown_flag.value:
            # Check if Nano is connected and update status
            if serial_handler.status.connected and not shared_memory.nano_connected.value:
                shared_memory.nano_connected.value = True
                print("📡 Nano communication active")
            
            # Update heartbeat monitor with latest Nano status
            if serial_handler.status.last_heartbeat_received > 0:
                heartbeat_monitor.update_nano_heartbeat()
            
            # Small sleep to prevent busy waiting
            time.sleep(0.01)  # 10ms
            
        print("🔴 Serial process stopping...")
        
    except KeyboardInterrupt:
        print("🛑 Serial process interrupted")
    except Exception as e:
        print(f"❌ Serial process error: {e}")
        # Ensure emergency stop is sent on critical errors
        serial_handler.send_emergency_stop()
    finally:
        # Cleanup
        serial_handler.stop()
        heartbeat_monitor.stop()
        print("✅ Serial process cleanup complete")

def setup_serial_communication() -> bool:
    """
    Setup function for serial communication system
    Called by the main process to initialize everything
    """
    try:
        print("⚙️ Setting up serial communication system...")
        
        # Test serial port availability
        import serial
        try:
            test_ser = serial.Serial(SERIAL_CONF.PORT, SERIAL_CONF.BAUDRATE, timeout=0.1)
            test_ser.close()
            print(f"✅ Serial port {SERIAL_CONF.PORT} is available")
        except Exception as e:
            print(f"❌ Serial port test failed: {e}")
            return False
        
        # Initialize shared memory values
        shared_memory.nano_connected.value = False
        shared_memory.system_ready.value = False
        
        print("✅ Serial communication setup complete")
        return True
        
    except Exception as e:
        print(f"❌ Serial communication setup failed: {e}")
        return False

# Import here to avoid circular imports
from config.serial_config import SERIAL_CONF
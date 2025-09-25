#!/usr/bin/env python3
"""
Complete fix script for WRO 2025 system
"""

import os
import sys

def fix_communication_init():
    """Fix the syntax error in communication/__init__.py"""
    init_file = "communication/__init__.py"
    
    correct_content = '''"""
Communication package for WRO 2025 vehicle
Handles all serial communication with Arduino Nano
"""

from .serial_handler import serial_handler, SerialHandler
from .heartbeat_monitor import heartbeat_monitor, HeartbeatMonitor

__all__ = ['serial_handler', 'SerialHandler', 'heartbeat_monitor', 'HeartbeatMonitor']
'''
    
    with open(init_file, 'w') as f:
        f.write(correct_content)
    print("✅ Fixed communication/__init__.py")

def create_missing_init_files():
    """Create missing __init__.py files if they don't exist"""
    init_files = {
        "core/__init__.py": '''"""
Core package for WRO 2025 vehicle
"""

from .process_manager import process_manager, ProcessManager
from .shared_memory import shared_memory, SharedMemoryManager
from .state_machine import state_machine, CompetitionStateMachine

__all__ = [
    'process_manager', 'ProcessManager',
    'shared_memory', 'SharedMemoryManager', 
    'state_machine', 'CompetitionStateMachine'
]''',
        
        "config/__init__.py": '''"""
Configuration package for WRO 2025 vehicle
"""

from .vehicle_config import RULES, VEHICLE, PERF, NANO_PROTOCOL
from .camera_config import CAMERA_CONFIG, PIPELINE_CONFIG, VISION_CONFIG
from .serial_config import SERIAL_CONF, PACKET_CONF, SAFETY_CONF

__all__ = [
    'RULES', 'VEHICLE', 'PERF', 'NANO_PROTOCOL',
    'CAMERA_CONFIG', 'PIPELINE_CONFIG', 'VISION_CONFIG',
    'SERIAL_CONF', 'PACKET_CONF', 'SAFETY_CONF'
]'''
    }
    
    for file_path, content in init_files.items():
        if not os.path.exists(file_path):
            with open(file_path, 'w') as f:
                f.write(content)
            print(f"✅ Created {file_path}")

def fix_process_manager_syntax():
    """Fix syntax error in process_manager.py"""
    file_path = "core/process_manager.py"
    
    with open(file_path, 'r') as f:
        content = f.read()
    
    # Fix the syntax error
    content = content.replace('timeout = 5.0  seconds', 'timeout = 5.0  # seconds')
    
    with open(file_path, 'w') as f:
        f.write(content)
    print("✅ Fixed syntax error in process_manager.py")

def main():
    """Main fix function"""
    print("🔧 Comprehensive WRO 2025 System Fix")
    
    fix_communication_init()
    create_missing_init_files()
    fix_process_manager_syntax()
    
    print("\n🎯 Fixes applied. Now run:")
    print("source wro2025_venv/bin/activate")
    print("python main.py")

if __name__ == "__main__":
    main()
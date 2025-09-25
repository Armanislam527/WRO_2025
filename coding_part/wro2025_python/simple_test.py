#!/usr/bin/env python3
"""
Simple test that avoids numpy dependencies
"""

import sys
import os

# Add the current directory to Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def test_basic_imports():
    """Test imports that don't require numpy"""
    print("🧪 Testing basic imports (no numpy)...")
    
    modules_to_test = [
        "config.vehicle_config",
        "config.serial_config", 
        "core.shared_memory",
        "core.state_machine",
        "communication.serial_handler",
        "communication.heartbeat_monitor",
        "utils.logger",
        "utils.performance_monitor"
    ]
    
    for module_name in modules_to_test:
        try:
            __import__(module_name)
            print(f"  ✅ {module_name}")
        except ImportError as e:
            print(f"  ❌ {module_name}: {e}")
        except Exception as e:
            if "numpy" in str(e):
                print(f"  ⚠️  {module_name}: Numpy issue (expected)")
            else:
                print(f"  ❌ {module_name}: {e}")

def test_shared_memory():
    """Test shared memory without numpy dependencies"""
    print("\n🧪 Testing shared memory...")
    
    try:
        from core.shared_memory import shared_memory, SensorData, VisionData
        
        # Test basic functionality
        sensor_data = SensorData()
        sensor_data.front_distance = 100
        shared_memory.update_sensor_data(sensor_data)
        
        retrieved = shared_memory.get_sensor_data()
        assert retrieved.front_distance == 100, "Sensor data mismatch"
        
        print("  ✅ Shared memory functional")
        return True
    except Exception as e:
        print(f"  ❌ Shared memory test failed: {e}")
        return False

def main():
    """Run basic tests"""
    print("=" * 50)
    print("🧪 WRO 2025 Basic System Test")
    print("=" * 50)
    
    test_basic_imports()
    test_shared_memory()
    
    print("\n📋 Basic tests completed.")
    print("💡 For full testing, use virtual environment with numpy installed")

if __name__ == "__main__":
    main()
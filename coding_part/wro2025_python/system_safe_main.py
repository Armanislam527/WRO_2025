#!/usr/bin/env python3
"""
WRO 2025 Main - System-safe version that avoids import issues
"""

import sys
import os

# Add current directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def setup_environment():
    """Setup that avoids problematic imports"""
    print("🔧 Setting up environment...")
    
    # Import only safe modules first
    from utils.logger import competition_logger
    competition_logger.log_system_start()
    
    return True

def main():
    """Main function that handles import errors gracefully"""
    print("🤖 WRO 2025 System - Safe Mode")
    
    try:
        if not setup_environment():
            print("❌ Setup failed")
            return
        
        # Try to import vision components with error handling
        try:
            from vision.vision_integrator import vision_integrator
            print("✅ Vision system available")
        except ImportError as e:
            print(f"⚠️  Vision system not available: {e}")
        
        # Try to import control components
        try:
            from control.navigation_controller import navigation_controller
            print("✅ Control system available")
        except ImportError as e:
            print(f"⚠️  Control system not available: {e}")
        
        print("\n🎯 System is running in safe mode.")
        print("💡 Some features may be limited due to import issues.")
        print("🔧 To fix: Use virtual environment or update system packages.")
        
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    main()
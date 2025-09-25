"""
Comprehensive logging system for WRO 2025 competition
Provides structured logging for debugging and performance analysis
"""

import logging
import os
import sys
from datetime import datetime
from typing import Dict, Any
from config.vehicle_config import PERF

class CompetitionLogger:
    """
    Advanced logging system for competition debugging and analysis
    """
    
    def __init__(self, log_level=logging.INFO):
        self.logger = None
        self.performance_log = []
        self.setup_complete = False
        
        # Create logs directory
        os.makedirs('logs', exist_ok=True)
        
        self.setup_logging(log_level)
    
    def setup_logging(self, log_level):
        """Setup comprehensive logging configuration"""
        try:
            # Create logger
            self.logger = logging.getLogger('WRO2025')
            self.logger.setLevel(log_level)
            
            # Remove existing handlers
            for handler in self.logger.handlers[:]:
                self.logger.removeHandler(handler)
            
            # Create formatter
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            
            # File handler
            log_filename = f"logs/wro2025_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
            file_handler = logging.FileHandler(log_filename)
            file_handler.setFormatter(formatter)
            self.logger.addHandler(file_handler)
            
            # Console handler
            console_handler = logging.StreamHandler(sys.stdout)
            console_handler.setFormatter(formatter)
            self.logger.addHandler(console_handler)
            
            self.setup_complete = True
            self.logger.info("Competition logging system initialized")
            
        except Exception as e:
            print(f"❌ Logging setup failed: {e}")
            # Fallback to basic print logging
            self.logger = None
    
    def log_system_start(self):
        """Log system startup"""
        if self.logger:
            self.logger.info("🤖 WRO 2025 Autonomous Vehicle System Starting")
            self.logger.info(f"📋 Configuration: {PERF.RESIZE_WIDTH}x{PERF.RESIZE_HEIGHT} @ {PERF.MAX_FPS}FPS")
        else:
            print("🤖 WRO 2025 Autonomous Vehicle System Starting")
    
    def log_state_transition(self, from_state: str, to_state: str, reason: str):
        """Log state machine transitions"""
        if self.logger:
            self.logger.info(f"🔄 State transition: {from_state} → {from_state}")
            self.logger.info(f"   Reason: {reason}")
        else:
            print(f"🔄 State transition: {from_state} → {to_state}")
    
    def log_vehicle_performance(self, performance_data: Dict[str, Any]):
        """Log vehicle performance metrics"""
        self.performance_log.append({
            'timestamp': datetime.now(),
            'data': performance_data
        })
        
        # Keep only recent logs
        if len(self.performance_log) > 1000:
            self.performance_log = self.performance_log[-1000:]
        
        if self.logger:
            self.logger.debug(f"📊 Performance: {performance_data}")
    
    def log_vision_data(self, vision_data: Dict[str, Any]):
        """Log vision system data"""
        if self.logger:
            self.logger.debug(f"👁️ Vision: {vision_data}")
    
    def log_control_decisions(self, control_data: Dict[str, Any]):
        """Log control system decisions"""
        if self.logger:
            self.logger.debug(f"🎮 Control: {control_data}")
    
    def log_emergency(self, emergency_type: str, details: str):
        """Log emergency situations"""
        if self.logger:
            self.logger.error(f"🚨 EMERGENCY: {emergency_type} - {details}")
        else:
            print(f"🚨 EMERGENCY: {emergency_type} - {details}")
    
    def log_competition_event(self, event_type: str, details: str):
        """Log competition-specific events"""
        if self.logger:
            self.logger.info(f"🎯 Competition: {event_type} - {details}")
        else:
            print(f"🎯 Competition: {event_type} - {details}")
    
    def save_performance_report(self):
        """Save performance report to file"""
        try:
            report_filename = f"logs/performance_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            
            with open(report_filename, 'w') as f:
                f.write("timestamp,cpu_usage,memory_usage,frame_rate,processing_time\n")
                for entry in self.performance_log:
                    data = entry['data']
                    f.write(f"{entry['timestamp']},"
                           f"{data.get('cpu_usage', 0)},"
                           f"{data.get('memory_usage', 0)},"
                           f"{data.get('frame_rate', 0)},"
                           f"{data.get('processing_time', 0)}\n")
            
            if self.logger:
                self.logger.info(f"📈 Performance report saved: {report_filename}")
                
        except Exception as e:
            if self.logger:
                self.logger.error(f"❌ Failed to save performance report: {e}")

# Global logger instance
competition_logger = CompetitionLogger()
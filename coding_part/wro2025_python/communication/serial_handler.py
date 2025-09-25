"""
Serial communication handler for Arduino Nano
Perfect protocol matching with robust error handling and heartbeat management
Optimized for Pi Zero 2W with minimal CPU overhead
"""

import serial
import time
import threading
from typing import Optional, Callable, Dict, Any
from dataclasses import dataclass
from config.serial_config import SERIAL_CONF, PACKET_CONF, SAFETY_CONF
from config.vehicle_config import NANO_PROTOCOL
from core.shared_memory import shared_memory, SensorData

@dataclass
class NanoStatus:
    """Current status of Nano communication"""
    connected: bool = False
    last_heartbeat_received: float = 0.0
    last_sensor_update: float = 0.0
    packets_sent: int = 0
    packets_received: int = 0
    errors: int = 0

class SerialHandler:
    """
    Handles all communication with Arduino Nano
    Implements exact protocol matching with robust error recovery
    """
    
    def __init__(self):
        self.serial_port: Optional[serial.Serial] = None
        self.status = NanoStatus()
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()
        
        # Callbacks for event handling
        self.callbacks: Dict[str, Callable] = {}
        
        # Buffer for incoming data
        self.rx_buffer = bytearray()
        self.last_tx_time = 0.0
        
    def initialize(self) -> bool:
        """Initialize serial connection to Nano"""
        try:
            print(f"🔌 Initializing serial connection to {SERIAL_CONF.PORT}...")
            
            with self.lock:
                self.serial_port = serial.Serial(
                    port=SERIAL_CONF.PORT,
                    baudrate=SERIAL_CONF.BAUDRATE,
                    timeout=SERIAL_CONF.TIMEOUT,
                    parity=SERIAL_CONF.PARITY,
                    stopbits=SERIAL_CONF.STOPBITS,
                    bytesize=SERIAL_CONF.BYTESIZE
                )
                
                # Allow time for port to initialize
                time.sleep(2.0)
                
                if self.serial_port.is_open:
                    print("✅ Serial port opened successfully")
                    
                    # Clear any existing data in buffers
                    self.serial_port.reset_input_buffer()
                    self.serial_port.reset_output_buffer()
                    
                    # Send initial heartbeat to establish connection
                    self._send_heartbeat()
                    
                    return True
                else:
                    print("❌ Failed to open serial port")
                    return False
                    
        except Exception as e:
            print(f"❌ Serial initialization error: {e}")
            return False
    
    def start(self) -> bool:
        """Start the serial communication thread"""
        if not self.initialize():
            return False
            
        self.running = True
        self.thread = threading.Thread(target=self._communication_loop, daemon=True)
        self.thread.start()
        
        print("🔄 Serial communication thread started")
        return True
    
    def stop(self) -> None:
        """Stop serial communication gracefully"""
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)
        
        with self.lock:
            if self.serial_port and self.serial_port.is_open:
                # Send emergency stop before closing
                self._send_emergency_stop()
                time.sleep(0.1)
                self.serial_port.close()
                
        print("🛑 Serial communication stopped")
    
    def send_motor_command(self, speed_percent: float) -> bool:
        """
        Send motor speed command to Nano
        speed_percent: -100% (full reverse) to +100% (full forward)
        """
        # Convert to Nano's expected format
        speed_byte = NANO_PROTOCOL.map_speed_to_nano(speed_percent)
        data = bytes([speed_byte])
        
        return self._send_packet(NANO_PROTOCOL.CMD_SET_MOTOR_SPEED, data)
    
    def send_servo_command(self, angle_degrees: float) -> bool:
        """
        Send servo angle command to Nano
        angle_degrees: 0-180 degrees (90 = center)
        """
        angle_byte = NANO_PROTOCOL.map_servo_angle(angle_degrees)
        data = bytes([angle_byte])
        
        return self._send_packet(NANO_PROTOCOL.CMD_SET_SERVO_ANGLE, data)
    
    def send_emergency_stop(self) -> bool:
        """Send emergency stop command to Nano"""
        print("🚨 Sending emergency stop to Nano")
        return self._send_packet(NANO_PROTOCOL.CMD_EMERGENCY_STOP, b'')
    
    def send_go_signal(self) -> bool:
        """Send GO signal to start mission (after start button pressed)"""
        print("🎯 Sending GO signal to Nano")
        return self._send_packet(NANO_PROTOCOL.CMD_PI_GO_SIGNAL, b'')
    
    def request_sensor_data(self) -> bool:
        """Explicitly request sensor data from Nano"""
        return self._send_packet(NANO_PROTOCOL.CMD_REQUEST_SENSOR_DATA, b'')
    
    def register_callback(self, event: str, callback: Callable) -> None:
        """Register callback for specific events"""
        self.callbacks[event] = callback
    
    def _send_packet(self, command: int, data: bytes) -> bool:
        """Send a packet to Nano with error handling"""
        with self.lock:
            if not self.serial_port or not self.serial_port.is_open:
                print("❌ Cannot send packet - serial port not open")
                return False
            
            try:
                packet = PACKET_CONF.create_packet(command, data)
                self.serial_port.write(packet)
                self.last_tx_time = time.time()
                self.status.packets_sent += 1
                return True
                
            except Exception as e:
                print(f"❌ Error sending packet: {e}")
                self.status.errors += 1
                return False
    
    def _send_heartbeat(self) -> bool:
        """Send heartbeat to keep Nano's watchdog happy"""
        return self._send_packet(NANO_PROTOCOL.CMD_HEARTBEAT, b'')
    
    def _parse_incoming_packet(self) -> Optional[Dict[str, Any]]:
        """Parse incoming data and extract complete packets"""
        if not self.serial_port:
            return None
            
        # Read available data
        try:
            data = self.serial_port.read(self.serial_port.in_waiting or 1)
            if data:
                self.rx_buffer.extend(data)
        except Exception as e:
            print(f"❌ Error reading serial data: {e}")
            return None
        
        # Look for complete packets in buffer
        while len(self.rx_buffer) >= 5:  # Minimum packet size
            # Find start byte
            start_index = -1
            for i in range(len(self.rx_buffer)):
                if self.rx_buffer[i] == NANO_PROTOCOL.PACKET_START_BYTE:
                    start_index = i
                    break
            
            if start_index == -1:
                # No start byte found, clear buffer
                self.rx_buffer.clear()
                break
            
            # Remove any garbage before start byte
            if start_index > 0:
                self.rx_buffer = self.rx_buffer[start_index:]
            
            if len(self.rx_buffer) < 5:  # Incomplete packet
                break
            
            # Extract packet components
            start_byte = self.rx_buffer[0]
            command = self.rx_buffer[1]
            length = self.rx_buffer[2]
            
            # Check if we have complete packet
            if len(self.rx_buffer) < 4 + length + 1:  # header + data + checksum
                break
            
            data_bytes = self.rx_buffer[3:3+length]
            received_checksum = self.rx_buffer[3+length]
            
            # Verify checksum
            header_data = bytes([command, length]) + data_bytes
            expected_checksum = PACKET_CONF.calculate_checksum(header_data)
            
            if received_checksum == expected_checksum:
                # Valid packet found
                packet = {
                    'command': command,
                    'data': bytes(data_bytes),
                    'timestamp': time.time()
                }
                
                # Remove processed packet from buffer
                self.rx_buffer = self.rx_buffer[4+length+1:]
                
                self.status.packets_received += 1
                return packet
            else:
                # Checksum error, skip this start byte and continue
                print("❌ Packet checksum error")
                self.rx_buffer = self.rx_buffer[1:]
                self.status.errors += 1
        
        return None
    
    def _handle_sensor_data(self, data: bytes) -> None:
        """Handle incoming sensor data from Nano"""
        try:
            if len(data) == 20:  # CompactSensorData struct size
                # Parse according to Nano's CompactSensorData structure
                sensor_data = SensorData()
                
                # Ultrasonic distances (uint16_t, big-endian)
                sensor_data.front_distance = (data[0] << 8) | data[1]
                sensor_data.right_distance = (data[2] << 8) | data[3]
                sensor_data.back_distance = (data[4] << 8) | data[5]
                sensor_data.left_distance = (data[6] << 8) | data[7]
                
                # IMU data (int16_t, big-endian)
                sensor_data.accel_x = self._bytes_to_int16(data[8], data[9])
                sensor_data.accel_y = self._bytes_to_int16(data[10], data[11])
                sensor_data.accel_z = self._bytes_to_int16(data[12], data[13])
                sensor_data.gyro_x = self._bytes_to_int16(data[14], data[15])
                sensor_data.gyro_y = self._bytes_to_int16(data[16], data[17])
                sensor_data.gyro_z = self._bytes_to_int16(data[18], data[19])
                
                sensor_data.timestamp = time.time()
                
                # Update shared memory
                shared_memory.update_sensor_data(sensor_data)
                self.status.last_sensor_update = time.time()
                
                # Notify callback if registered
                if 'sensor_data' in self.callbacks:
                    self.callbacks['sensor_data'](sensor_data)
                    
            else:
                print(f"⚠️  Unexpected sensor data length: {len(data)}")
                
        except Exception as e:
            print(f"❌ Error parsing sensor data: {e}")
            self.status.errors += 1
    
    def _handle_start_ack(self, data: bytes) -> None:
        """Handle start acknowledgment from Nano"""
        if data and len(data) >= 1:
            started = bool(data[0])
            print(f"📢 Nano start acknowledgment: {'STARTED' if started else 'NOT STARTED'}")
            
            if 'start_ack' in self.callbacks:
                self.callbacks['start_ack'](started)
    
    def _handle_error(self, data: bytes) -> None:
        """Handle error messages from Nano"""
        error_code = data[0] if data else 0
        print(f"❌ Nano reported error: 0x{error_code:02X}")
        
        if 'error' in self.callbacks:
            self.callbacks['error'](error_code)
    
    def _bytes_to_int16(self, high_byte: int, low_byte: int) -> int:
        """Convert two bytes to signed int16 (big-endian)"""
        value = (high_byte << 8) | low_byte
        if value & 0x8000:  # Check sign bit
            value -= 0x10000
        return value
    
    def _communication_loop(self) -> None:
        """Main communication loop running in separate thread"""
        print("🔄 Starting serial communication loop...")
        
        last_heartbeat_time = 0.0
        last_sensor_request_time = 0.0
        connection_retries = 0
        
        while self.running:
            current_time = time.time()
            
            try:
                # Reconnect if necessary
                if not self.status.connected:
                    if self.initialize():
                        self.status.connected = True
                        connection_retries = 0
                        shared_memory.nano_connected.value = True
                        print("✅ Nano connection established")
                    else:
                        connection_retries += 1
                        if connection_retries % 10 == 0:  # Log every 10 retries
                            print(f"⚠️  Nano connection retry #{connection_retries}")
                        time.sleep(1.0)
                        continue
                
                # Send heartbeat regularly
                if current_time - last_heartbeat_time > SERIAL_CONF.HEARTBEAT_INTERVAL_MS / 1000.0:
                    if self._send_heartbeat():
                        last_heartbeat_time = current_time
                
                # Request sensor data regularly
                if current_time - last_sensor_request_time > SERIAL_CONF.SENSOR_REQUEST_INTERVAL_MS / 1000.0:
                    if self.request_sensor_data():
                        last_sensor_request_time = current_time
                
                # Parse incoming packets
                packet = self._parse_incoming_packet()
                if packet:
                    self.status.last_heartbeat_received = current_time
                    
                    # Handle based on command type
                    if packet['command'] == NANO_PROTOCOL.CMD_SENSOR_DATA:
                        self._handle_sensor_data(packet['data'])
                    elif packet['command'] == NANO_PROTOCOL.CMD_ALL_SENSOR_DATA_COMPACT:
                        self._handle_sensor_data(packet['data'])
                    elif packet['command'] == NANO_PROTOCOL.CMD_START_ACK:
                        self._handle_start_ack(packet['data'])
                    elif packet['command'] == NANO_PROTOCOL.CMD_ERROR:
                        self._handle_error(packet['data'])
                    else:
                        print(f"⚠️  Unknown command received: 0x{packet['command']:02X}")
                
                # Check for communication timeout
                if current_time - self.status.last_heartbeat_received > SAFETY_CONF.COMMUNICATION_TIMEOUT_MS / 1000.0:
                    if self.status.connected:
                        print("❌ Nano communication timeout")
                        self.status.connected = False
                        shared_memory.nano_connected.value = False
                
                # Small delay to prevent CPU overload
                time.sleep(0.001)  # 1ms
                
            except Exception as e:
                print(f"❌ Communication loop error: {e}")
                self.status.connected = False
                shared_memory.nano_connected.value = False
                time.sleep(0.5)  # Wait before retry
        
        print("🔴 Serial communication loop stopped")

# Global serial handler instance
serial_handler = SerialHandler()
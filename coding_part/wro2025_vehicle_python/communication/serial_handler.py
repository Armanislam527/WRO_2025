# communication/serial_handler.py

"""Handles serial communication with the Arduino Nano, matching the C++ protocol."""

import serial
import time
import threading
from typing import Optional, Callable
import logging
from sensors.sensor_data import SensorData, unpack_compact_sensor_data, COMPACT_SENSOR_DATA_SIZE

# Configure logging
logging.basicConfig(level=logging.INFO) # Will be replaced by utils logger later
logger = logging.getLogger(__name__)

# --- Serial Protocol Constants (from C++ logtext.txt) ---
# Commands FROM Pi TO Nano
CMD_SET_MOTOR_SPEED = 0x01
CMD_SET_SERVO_ANGLE = 0x02
CMD_EMERGENCY_STOP = 0x03
CMD_HEARTBEAT = 0x04
CMD_REQUEST_SENSOR_DATA = 0x05 # Pi can request data
CMD_PI_GO_SIGNAL = 0x06 # Pi's GO command

# Commands FROM Nano TO Pi
CMD_SENSOR_DATA = 0x10 # Standard sensor data packet
CMD_START_ACK = 0x11 # Nano acknowledges start button press
CMD_ERROR = 0x12 # Nano sends an error code
CMD_ALL_SENSOR_DATA_COMPACT = 0x13 # Nano sends the compact struct

PACKET_START_BYTE = 0xAA
PACKET_MAX_LENGTH = 32 # As defined in C++

class SerialHandler:
    """
    Manages serial communication with the Nano, matching the C++ protocol.
    Handles sending commands and receiving sensor data packets.
    """

    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn: Optional[serial.Serial] = None
        self.is_open = False
        self._stop_receiving = threading.Event()
        self._receive_thread: Optional[threading.Thread] = None

        # Data storage
        self._latest_sensor_data = SensorData()
        self._data_lock = threading.Lock()
        self._start_ack_received = False
        self._start_ack_lock = threading.Lock()
        self._last_error_code = 0
        self._error_lock = threading.Lock()

        # Buffers and parser state
        self._rx_buffer = bytearray()
        self._parse_state = "IDLE" # IDLE, COMMAND_RECEIVED, LENGTH_RECEIVED, DATA_RECEIVED
        self._current_command = 0
        self._current_length = 0
        self._data_bytes_received = 0

        # Callbacks (optional)
        self.on_sensor_data_updated: Optional[Callable[[SensorData], None]] = None
        self.on_start_ack_received: Optional[Callable[[], None]] = None
        self.on_error_received: Optional[Callable[[int], None]] = None # Callback for CMD_ERROR

    def open(self) -> bool:
        """Open the serial port."""
        if self.is_open:
            logger.warning("Serial port already open.")
            return False

        try:
            # Open serial connection
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1  # Read timeout in seconds
            )
            self.is_open = True
            logger.info(f"Serial port {self.port} opened successfully.")
            
            # Start the receiving thread
            self._stop_receiving.clear()
            self._receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self._receive_thread.start()
            logger.debug("Serial receive thread started.")
            return True
            
        except serial.SerialException as e:
            logger.error(f"Failed to open serial port {self.port}: {e}")
            self.is_open = False
            self.serial_conn = None
            return False

    def close(self):
        """Close the serial port and stop the receiving thread."""
        if not self.is_open:
            return

        logger.info("Closing serial port...")
        self._stop_receiving.set() # Signal receive thread to stop

        # Wait for the receive thread to finish
        if self._receive_thread and self._receive_thread.is_alive():
            self._receive_thread.join(timeout=2.0) # Wait up to 2 seconds
            if self._receive_thread.is_alive():
                logger.warning("Serial receive thread did not stop gracefully.")

        # Close the serial connection
        if self.serial_conn:
            self.serial_conn.close()
            self.serial_conn = None
            
        self.is_open = False
        logger.info("Serial port closed.")

    def _receive_loop(self):
        """Background thread function to receive and parse data."""
        logger.debug("Serial receive loop started.")
        
        while not self._stop_receiving.is_set():
            try:
                if self.serial_conn and self.serial_conn.in_waiting > 0:
                    bytes_available = self.serial_conn.in_waiting
                    logger.debug(f"SerialHandler: Bytes available to read: {bytes_available}")
                    raw_data = self.serial_conn.read(self.serial_conn.in_waiting)
                    if raw_data:
                        logger.debug(f"SerialHandler: Received {len(raw_data)} bytes.") # Add this line
                        # Process each byte sequentially like C++
                        for byte_val in raw_data:
                            self._process_byte(byte_val)
                else:
                    # Small sleep to prevent busy-waiting
                    time.sleep(0.001) # 1ms sleep
                    
            except serial.SerialException as e:
                logger.error(f"Serial read error: {e}")
                if "device disconnected" in str(e).lower():
                    logger.error("Serial device appears disconnected. Stopping receive loop.")
                    break
            except Exception as e:
                logger.error(f"Unexpected error in receive loop: {e}", exc_info=True)

        logger.debug("SerialHandler: Serial receive loop stopped.")

    def _process_byte(self, incoming_byte: int):
        """Process a single incoming byte according to the state machine."""
        logger.debug(f"SerialHandler: Processing byte: 0x{incoming_byte:02X}, State: {self._parse_state}")
        if self._parse_state == "IDLE":
            if incoming_byte == PACKET_START_BYTE:
                logger.debug(f"SerialHandler: Found packet start byte 0xAA") # Add this line
                self._parse_state = "COMMAND_RECEIVED"
            # else: Ignore byte, stay in IDLE

        elif self._parse_state == "COMMAND_RECEIVED":
            self._current_command = incoming_byte
            logger.debug(f"SerialHandler: Received command byte: 0x{self._current_command:02X}")
            self._parse_state = "LENGTH_RECEIVED"

        elif self._parse_state == "LENGTH_RECEIVED":
            self._current_length = incoming_byte
            self._data_bytes_received = 0
            self._rx_buffer.clear() # Clear buffer for new data
            logger.debug(f"SerialHandler: Received length byte: {self._current_length}")
            if self._current_length == 0:
                self._parse_state = "DATA_RECEIVED" # No data, go straight to checksum
            elif self._current_length > PACKET_MAX_LENGTH:
                logger.warning(f"Packet length {self._current_length} exceeds maximum {PACKET_MAX_LENGTH}. Discarding.")
                self._parse_state = "IDLE" # Reset state
            else:
                self._parse_state = "RECEIVING_DATA"

        elif self._parse_state == "RECEIVING_DATA":
            if self._data_bytes_received < self._current_length:
                self._rx_buffer.append(incoming_byte)
                self._data_bytes_received += 1
                if self._data_bytes_received == self._current_length:
                    logger.debug(f"SerialHandler: All {self._current_length} data bytes received.")
                    self._parse_state = "DATA_RECEIVED"
            # else: Ignore extra bytes if we somehow receive more than length

        elif self._parse_state == "DATA_RECEIVED":
            # incoming_byte is the checksum
            received_checksum = incoming_byte
            logger.debug(f"SerialHandler: Checksum byte received: 0x{received_checksum:02X}")
            # Prepare packet for checksum calculation: [START][CMD][LEN][DATA...]
            packet_for_checksum = bytearray([PACKET_START_BYTE, self._current_command, self._current_length])
            packet_for_checksum.extend(self._rx_buffer)
            
            # Calculate checksum
            calculated_checksum = 0
            for b in packet_for_checksum:
                calculated_checksum ^= b
            logger.debug(f"SerialHandler: Calculated checksum: 0x{calculated_checksum:02X}")
            if calculated_checksum == received_checksum:
                # Valid packet, handle it
                logger.debug(f"SerialHandler: Valid packet received: CMD=0x{self._current_command:02X}, LEN={self._current_length}")
                self._handle_valid_packet(self._current_command, bytes(self._rx_buffer))
            else:
                logger.warning(f"SerialHandler: Checksum mismatch for command 0x{self._current_command:02X}. Packet discarded.(Rxd:0x{received_checksum:02X}, Calc:0x{calculated_checksum:02X})")
            
            # Reset parser state regardless of checksum validity
            self._parse_state = "IDLE"
            self._rx_buffer.clear()
            self._data_bytes_received = 0

    def _handle_valid_packet(self, command: int, data_bytes: bytes):
        """Handle a received packet with a valid checksum."""
        logger.debug(f"SerialHandler: Handling valid packet: CMD=0x{command:02X}, LEN={len(data_bytes)}")
        
        if command == CMD_START_ACK and len(data_bytes) >= 1:
            started_flag = data_bytes[0]
            if started_flag == 0x01: # C++ sends 0x01 if started
                logger.info("SerialHandler: Received START_ACK (button pressed) from Nano.")
                with self._start_ack_lock:
                    self._start_ack_received = True
                # Trigger callback if set
                if self.on_start_ack_received:
                    try:
                        self.on_start_ack_received()
                    except Exception as e:
                        logger.error(f"SerialHandler: Error in on_start_ack_received callback: {e}")
            else:
                logger.debug(f"SerialHandler: Received START_ACK with flag 0x{started_flag:02X} (not started).")

        elif command == CMD_ALL_SENSOR_DATA_COMPACT and len(data_bytes) == COMPACT_SENSOR_DATA_SIZE:
            logger.debug("SerialHandler: Received COMPACT_SENSOR_DATA packet.")
            # The C++ code packs the struct directly. We need to append the checksum byte
            # that was used for internal validation in the unpack function.
            # However, our unpack function expects the full packet including checksum.
            # Let's reconstruct the packet data for unpacking.
            # Actually, our previous unpack function expected the raw data + checksum byte.
            # But here, data_bytes is just the data part. We need to pass it correctly.
            # Let's modify the unpack function call or reconstruct the packet.
            # Better: Modify unpack function to take just the data part.
            # Or, pass data_bytes as is, assuming it's the struct data.
            # The C++ code casts the data part directly to the struct.
            # Our unpack function should take data_bytes directly.
            sensor_data, is_valid_internal = unpack_compact_sensor_data(data_bytes)
            if is_valid_internal: # Checksum inside the struct data
                logger.debug(f"SerialHandler: Unpacked sensor data: F={sensor_data.us_data.front_distance:.1f}cm,"f"GX={sensor_data.imu_data.gyro_x:.2f}d/s,"f" GY={sensor_data.imu_data.gyro_y:.2f}d/s,"f" GZ={sensor_data.imu_data.gyro_z:.2f}d/s")
                with self._data_lock:
                    self._latest_sensor_data = sensor_data
                # Trigger callback if set
                if self.on_sensor_data_updated:
                    try:
                        self.on_sensor_data_updated(sensor_data)
                    except Exception as e:
                        logger.error(f"SerialHandler: Error in on_sensor_data_updated callback: {e}")
            else:
                logger.warning("SerialHandler: Received sensor data packet with invalid internal checksum.")

        elif command == CMD_ERROR and len(data_bytes) >= 1:
            error_code = data_bytes[0]
            logger.warning(f"SerialHandler: Received ERROR code 0x{error_code:02X} from Nano.")
            with self._error_lock:
                self._last_error_code = error_code
            if self.on_error_received:
                try:
                    self.on_error_received(error_code)
                except Exception as e:
                    logger.error(f"SerialHandler: Error in on_error_received callback: {e}")

        elif command == CMD_SENSOR_DATA:
            # Handle standard sensor data packet if needed (not used in C++?)
            logger.debug(f"SerialHandler: Received standard SENSOR_DATA packet (len={len(data_bytes)}). Not implemented handler.")
        else:
            logger.debug(f"SerialHandler: Received unhandled packet: CMD=0x{command:02X}, LEN={len(data_bytes)}")

    # --- Sending Commands ---

    def _send_packet(self, command: int, data: bytes = b''):
        """Internal method to create and send a packet."""
        if not self.is_open or not self.serial_conn:
            logger.warning("SerialHandler: Cannot send packet, serial port is not open.")
            return False

        length = len(data)
        if length > PACKET_MAX_LENGTH:
            logger.error(f"SerialHandler: Data length {length} exceeds maximum ({PACKET_MAX_LENGTH} bytes).")
            return False

        packet = bytearray()
        packet.append(PACKET_START_BYTE)
        packet.append(command)
        packet.append(length)
        packet.extend(data)

        # Calculate checksum (XOR of all bytes: start, cmd, len, data)
        checksum = 0
        for b in packet:
            checksum ^= b
        packet.append(checksum)

        try:
            logger.debug(f"SerialHandler: Sending packet: START=0xAA, CMD=0x{command:02X}, LEN={length},DATA_SIZE={len(data)}, DATA={data.hex()}, CHECKSUM=0x{checksum:02X}")
            self.serial_conn.write(packet)
            # logger.debug(f"Sent packet: {packet.hex()}")
            return True
        except serial.SerialException as e:
            logger.error(f"SerialHandler: Failed to send packet: {e}")
            return False

    # --- Public Methods for Sending Commands (Matching C++ names) ---

    def sendGoCommand(self) -> bool:
        """Send the Pi's GO command to the Nano."""
        logger.info("SerialHandler: Sending GO command to Nano.")
        return self._send_packet(CMD_PI_GO_SIGNAL)

    def sendHeartbeat(self) -> bool:
        """Send a heartbeat packet."""
        logger.debug("SerialHandler: Sending heartbeat.")
        return self._send_packet(CMD_HEARTBEAT)

    def sendEmergencyStop(self) -> bool:
        """Send an emergency stop command."""
        logger.warning("SerialHandler: Sending EMERGENCY STOP command to Nano.")
        return self._send_packet(CMD_EMERGENCY_STOP)

    def sendMotorSpeedCommand(self, speed: int) -> bool:
        """
        Send motor speed command.
        Speed: int8_t mapped to -128 to 127 (like C++ map function).
        """
        # Clamp speed to int8_t range
        clamped_speed = max(-128, min(127, speed))
        # Convert signed int to unsigned byte for transmission
        speed_byte = clamped_speed & 0xFF
        logger.debug(f"SerialHandler: Sending motor speed command: {speed} (byte: 0x{speed_byte:02X})")
        return self._send_packet(CMD_SET_MOTOR_SPEED, bytes([speed_byte]))

    def sendServoAngleCommand(self, angle: int) -> bool:
        """
        Send servo angle command.
        Angle: uint8_t (0-180).
        """
        clamped_angle = max(0, min(180, angle))
        logger.debug(f"SerialHandler: Sending servo angle command: {angle}")
        return self._send_packet(CMD_SET_SERVO_ANGLE, bytes([clamped_angle]))

    def requestData(self) -> bool:
        """Request sensor data from Nano (if Nano supports CMD_REQUEST_SENSOR_DATA)."""
        logger.debug("SerialHandler: Requesting sensor data from Nano.")
        return self._send_packet(CMD_REQUEST_SENSOR_DATA)


    # --- Receiving Data ---

    def getLatestSensorData(self) -> SensorData:
        """Get the latest sensor data received."""
        with self._data_lock:
            logger.debug(f"SerialHandler: getLatestSensorData called, returning data: F={self._latest_sensor_data.us_data.front_distance:.1f}cm,"f"GX={self._latest_sensor_data.imu_data.gyro_x:.2f}d/s,"f" GY={self._latest_sensor_data.imu_data.gyro_y:.2f}d/s,"f" GZ={self._latest_sensor_data.imu_data.gyro_z:.2f}d/s")
            # Returning reference for now, as in C++
            return self._latest_sensor_data

    def hasStartAck(self) -> bool:
        """Check if the START_ACK has been received."""
        with self._start_ack_lock:
            logger.debug(f"SerialHandler: hasStartAck called, returning {self._start_ack_received}")
            return self._start_ack_received

    def acknowledgeStartAck(self):
        """Clear the START_ACK received flag."""
        with self._start_ack_lock:
            logger.debug(f"SerialHandler: acknowledgeStartAck called, clearing flag: {self._start_ack_received}")
            self._start_ack_received = False
        logger.debug("START_ACK acknowledged and cleared.")

    def getLastError(self) -> int:
        """Get the last error code received."""
        with self._error_lock:
            return self._last_error_code

# Example usage (if run as script)
if __name__ == "__main__":
    import config.vehicle_config as cfg
    
    def on_sensor_data( SensorData):
        # Simple print, avoid flooding console
        pass
        # print(f"Callback - F={data.us_data.front_distance:.1f}cm, GZ={data.imu_data.gyro_z:.2f}d/s")

    def on_start():
        print("Callback - START ACK (Button Pressed) Received!")

    def on_error(error_code: int):
        print(f"Callback - ERROR 0x{error_code:02X} Received!")

    sh = SerialHandler(cfg.SERIAL_PORT, cfg.SERIAL_BAUDRATE)
    sh.on_sensor_data_updated = on_sensor_data
    sh.on_start_ack_received = on_start
    sh.on_error_received = on_error

    if sh.open():
        print("Serial port opened. Waiting for data or sending commands...")
        try:
            # Example: Send a heartbeat periodically and check for ack
            counter = 0
            while True:
                time.sleep(1)
                counter += 1
                # sh.sendHeartbeat() # Uncomment if you want to send heartbeats
                if counter % 5 == 0: # Every 5 seconds
                    print(f"Status: StartAck={sh.hasStartAck()}")
                    if sh.hasStartAck():
                        print("Sending GO command!")
                        sh.sendGoCommand()
                        sh.acknowledgeStartAck() # Clear the flag
                        
        except KeyboardInterrupt:
            print("\nInterrupted by user.")
        finally:
            sh.close()
    else:
        print("Failed to open serial port.")

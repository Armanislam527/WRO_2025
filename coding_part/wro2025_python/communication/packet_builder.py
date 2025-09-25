# communication/packet_builder.py
class PacketBuilder:
    @staticmethod
    def build_motor_command(speed: int) -> bytes:
        """Match Nano's expected protocol: [0xAA, CMD, LEN, DATA, CHECKSUM]"""
        # Convert speed to match Nano's mapping (-128 to 127 → -255 to 255)
        speed_byte = int((speed / 255) * 127)
        data = bytes([speed_byte])
        return PacketBuilder._create_packet(0x01, data)  # CMD_SET_MOTOR_SPEED
    
    @staticmethod
    def build_sensor_request() -> bytes:
        """Request sensor data from Nano"""
        return PacketBuilder._create_packet(0x05, b'')  # CMD_REQUEST_SENSOR_DATA